#pragma once

#define DEBUG

#if defined(DEBUG) && !defined(NODEBUG)
#define DEBUGS(s)  do{Serial.println(s)}while(0)

#else
#define DEBUGS(s)   
#endif