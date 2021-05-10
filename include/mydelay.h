/*mydelay*/

#ifndef _MYDELAY_H
#define _MYDELAY_H

// myDelay
void myDelay(int iMyDel) {
  unsigned long int uiMyDelMillis;

  uiMyDelMillis = millis();
  while ( (millis() - uiMyDelMillis) < iMyDel)
  {}

}

#endif