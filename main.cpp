#include "stm8s.h"

class TouchController;
class TouchPad;

unsigned long TimeCount=0;

void CLK_16mHzSYSCLKConfig(void)
{
  unsigned char CLK_Prescaler = 0;

  CLK->CKDIVR &= (uint8_t)(~0x18);
  CLK->CKDIVR |= (uint8_t)((uint8_t)CLK_Prescaler & (uint8_t)0x18);
}

class TouchPad {
private:
  TouchPad * _nextPad;
  unsigned char _pinMask;
  unsigned short _timerValue;
  unsigned short _levelValue;
  signed short _driff;
  signed short _sumValue;
  
  bool state;
  
  friend class TouchController;
  TouchController* _controller;
  
  
public:
  TouchPad (TouchController &controller, unsigned char pin);
  bool getState (void);
  void setState (bool st) { state = st; }
  void setLevel (unsigned short value) {_levelValue = value;}
};

bool TouchPad::getState () {
  short diff = _timerValue - _levelValue;
  if ( diff > 2 || diff < -2) {
    
    _sumValue += diff>0? 1:-1;
    
    if (_sumValue > 10) {
      if (_timerValue - _levelValue > 5){
        state = true;
        _levelValue = _timerValue;
        _sumValue = 0;
      }
    }
    else if (_sumValue < -10) {
      if (_levelValue - _timerValue > 5){
        state = false;
        _levelValue = _timerValue;
        _sumValue = 0;
      }
    }
  }
  else {
    _sumValue = 0;
    _driff += diff;
    if (_driff > 20000) {
      _levelValue += 1;
      _driff = 0;
    }
    else if (_driff < -15000) {
      _levelValue -= 1;
      _driff = 0;
    }
  }
  
  return state;
}

class TouchController {
private:
  
  GPIO_TypeDef *_port;
  unsigned char _pinsMask;
  
  friend class TouchPad;
  TouchPad * _firstPad;
  TouchPad * _lastPad;
  
#define TOUCH_TIMER TIM2
  
public:
  unsigned char totalPad;
  unsigned char meassuredPad;
  
  TouchController ( GPIO_TypeDef *port): _port(port), _pinsMask(0), totalPad(0)
  {
    TOUCH_TIMER->ARRH = 0xff; // reload value
    TOUCH_TIMER->ARRL = 0xff;
    TOUCH_TIMER->IER |= TIM2_IER_UIE; // update interrupt enable 
  }
  
  void startMeassure (void)
  {
    _port->DDR &= ~_pinsMask;
    _port->CR1 &= ~_pinsMask;
    _port->CR2 |= _pinsMask;
    
    TOUCH_TIMER->CNTRH = 0;
    TOUCH_TIMER->CNTRL = 0;
    TOUCH_TIMER->CR1 = TIM2_CR1_CEN; //Counter Enable mask
    TOUCH_TIMER->ARRL = 0xff;
    meassuredPad=0;
  }
  void stopMeassure (void)
  {
    TOUCH_TIMER->CR1 &= ~TIM2_CR1_CEN;
    
    _port->DDR |= _pinsMask;
    _port->CR1 |= _pinsMask;
    _port->CR2 |= _pinsMask;
    _port->ODR |= _pinsMask;
    
    TouchPad * currentPad = _firstPad;
    while (currentPad) {
      currentPad->getState();
      currentPad = currentPad->_nextPad;
    }
  }
  
  void addPad ( TouchPad *pad);
  
  void captureInterrupt (void);
  
  void calibrateZero (void);
};

void TouchController::calibrateZero (void) {
  for (int i = 50; i>0; i--){
    startMeassure();
    // wait for measure to complete
    while (meassuredPad < totalPad);
  }
  
  TouchPad * currentPad = _firstPad;
  while (currentPad) {
    currentPad->setLevel(currentPad->_timerValue);
    currentPad->setState (false);
    currentPad = currentPad->_nextPad;
  }
}

void TouchController::addPad ( TouchPad *pad){
  if (!_firstPad) {
    _firstPad = pad;
    _pinsMask |= pad->_pinMask;
    totalPad++;
  }
  else {
    TouchPad * currentPad = _firstPad;
    do {
      if (currentPad->_nextPad) {
        currentPad = currentPad->_nextPad;
        continue;
      }
      else {
        currentPad->_nextPad = pad;
        _pinsMask |= pad->_pinMask;
        totalPad++;
      }
    } while (true);
  }
}
  
void TouchController::captureInterrupt (void){
  // read timer value;
  unsigned short timerValue = (unsigned short)(TOUCH_TIMER->CNTRH);
  timerValue = (timerValue<<8) | (TOUCH_TIMER->CNTRL);
  
  unsigned char portValue = _port->IDR;
  TouchPad * currentPad = _firstPad;
  
  // time out, take all pin = 0, and stop meassure
  if (TOUCH_TIMER->SR1 & TIM4_SR1_UIF) {
    portValue = 0;
    TOUCH_TIMER->SR1 &= ~TIM2_SR1_UIF;
    stopMeassure();
  }
  
  while (currentPad) {
    if (currentPad->_pinMask ^ portValue)
    {
      currentPad->_timerValue = (currentPad->_timerValue + timerValue)>>1;
      if ((++meassuredPad) >= totalPad){
        stopMeassure();
      }
    }
    currentPad = currentPad->_nextPad;
  }

}

TouchPad::TouchPad (TouchController &controller, unsigned char pin): \
            _nextPad(0),
            _pinMask(1<<pin),
            _controller(&controller)
            
{
  _controller->_port->DDR |= _pinMask; // output mode
  _controller->_port->CR1 |= _pinMask; // push pull
  _controller->_port->CR2 |= _pinMask; // fast
  _controller->_port->ODR |= _pinMask; // high
}


TouchController touchCtl(GPIOC);
TouchPad pad(touchCtl,1);

int main()
{
  CLK_16mHzSYSCLKConfig();
  
  touchCtl.addPad(&pad);
  
  // setup control pin for cap sense
  GPIOC->DDR = 12; // output
  GPIOC->CR1 = 12; // push pull
  GPIOC->CR2 = 12; // fast mode
  GPIOC->ODR = 8;
  
  GPIOD->DDR = 0xff;
  GPIOD->CR1 = 0xff;
  GPIOD->ODR = 0xff;
  
  // external interrupt port c falling edge 
  EXTI->CR1 = (2<<4);
  
  TIM4->ARR = 250; //auto-reload register
  TIM4->PSCR = 6; //prescaler register
  TIM4->IER |= TIM4_IER_UIE; // update interrupt enable 
  
  enableInterrupts();
  
  TIM4->CR1 = TIM4_CR1_CEN;
  
  touchCtl.calibrateZero();
  
  while (true){
    //wfi();
    unsigned long t = TimeCount;
    while (t == TimeCount); // delay 1ms
    
    touchCtl.startMeassure();
    
    // wait for measure to complete
    while (touchCtl.meassuredPad < touchCtl.totalPad);
    
    if (pad.getState() == true)
      GPIOD->ODR = 0;
    else GPIOD->ODR = 0xff;

  }

}

void GPIOB_interruptHandler (void){
  //GPIOD->ODR = GPIOB->IDR;
}
void GPIOC_interruptHandler (void){
  touchCtl.captureInterrupt();
}
void timer2_interrupt (void){
  touchCtl.captureInterrupt();
}

void timer4_interrupt (void){
  TIM4->SR1 &= ~TIM4_SR1_UIF;
  TimeCount++;
}

void assert_failed(uint8_t* file, uint32_t line){}