#ifndef DEBUG_H_
#define DEBUG_H_

// Define debug and log port
#define DEBUG_PORT

// Define wanted debug levels
#define LOG_ENABLED
#define TRACE_ENABLED
#define ERRORS_ENABLED
#define DEBUG_ENABLED



#ifdef DEBUG_PORT
  #define DebugInit(baud) Serial.begin(baud)
  #define LogInit(baud)   Serial.begin(baud)
#else
  #define DebugInit(baud)
  #define LogInit(baud)
#endif

/******************** Debug levels ********************/
#ifdef LOG_ENABLED
  #define Log(...)        Serial.printf( __VA_ARGS__ )
  #define LogFunc(...) do { Serial.print("["); Serial.print(__PRETTY_FUNCTION__); Serial.print("] "); Serial.printf(__VA_ARGS__); } while (0)
  #define Logln(...)      Serial.println( __VA_ARGS__ )
#else
  #define Log(...)
  #define LogFunc(...)
  #define Logln(...)
#endif

#ifdef TRACE_ENABLED
  #define Trace(...)        Serial.printf( __VA_ARGS__ )
  #define TraceFunc(...) do { Serial.print("["); Serial.print(__PRETTY_FUNCTION__); Serial.print("] "); Serial.printf(__VA_ARGS__); } while (0)
  #define Traceln(...)      Serial.println( __VA_ARGS__ )
#else
  #define Trace(...)
  #define TraceFunc(...)
  #define Traceln(...)
#endif

#ifdef ERRORS_ENABLED
  #define Err(...)      Serial.printf( __VA_ARGS__ )
  #define ErrFunc(...) do { Serial.print("["); Serial.print(__PRETTY_FUNCTION__); Serial.print("] "); Serial.printf(__VA_ARGS__); } while (0)
  #define Errln(...)     Serial.println( __VA_ARGS__ )
#else
  #define Err(...)
  #define ErrFunc(...)
  #define Errln(...)
#endif

#ifdef DEBUG_ENABLED
  #define Debugf(...)      Serial.printf( __VA_ARGS__ )
#else
  #define Debugf(...)
#endif

#endif // DEBUG_H_