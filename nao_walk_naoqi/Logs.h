#ifndef LOGS_H 
#define LOGS_H

#define LOG_ERR(...)  { fprintf(stderr,__VA_ARGS__); fprintf(stderr,"\n"); }
#define LOG_INF(...) { fprintf(stdout,__VA_ARGS__) ; fprintf(stdout,"\n"); }

#ifdef DEBUG
    #define LOG_DBG(...) { fprintf(stderr,__VA_ARGS__) ; fprintf(stderr,"\n"); }
#else
    #define LOG_DBG(...)
#endif
#endif
