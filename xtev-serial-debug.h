#ifndef _XTEV_SERIAL_DEBUG_H_
#define _XTEV_SERIAL_DEBUG_H_

#define XTEV_DEBUG

#ifdef XTEV_DEBUG
#define LEVEL_DEBUG             0
#define LEVEL_INFO              1
#define LEVEL_WARNING           2
#define LEVEL_ERROR             3

#define CURRENT_LEVEL           LEVEL_DEBUG

#define xtev_debug(file, fmt, ...) \
do{ \
    if(CURRENT_LEVEL <= LEVEL_DEBUG) { \
        printf("%s +%d: %s: DEBUG: "fmt"\n", file, __LINE__, __FUNCTION__, ##__VA_ARGS__); \
    } \
}while(0)

#define xtev_info(file, fmt, ...) \
do{ \
    if(CURRENT_LEVEL <= LEVEL_INFO) { \
        printf("%s +%d: %s: INFO: "fmt"\n", file, __LINE__, __FUNCTION__, ##__VA_ARGS__); \
    } \
}while(0)

#define xtev_warning(file, fmt, ...) \
do{ \
    if(CURRENT_LEVEL <= LEVEL_WARNING) { \
        printf("%s +%d: %s: WARNING: "fmt"\n", file, __LINE__, __FUNCTION__, ##__VA_ARGS__); \
    } \
}while(0)

#define xtev_error(file, fmt, ...) \
do{ \
    if(CURRENT_LEVEL <= LEVEL_ERROR) { \
        printf("%s +%d: %s: ERROR: "fmt"\n", file, __LINE__, __FUNCTION__, ##__VA_ARGS__); \
    } \
}while(0)
#else
#define xtev_debug(file, fmt, ...)
#define xtev_info(file, fmt, ...)
#define xtev_warning(file, fmt, ...)
#define xtev_error(file, fmt, ...)
#endif/* XTEV_DEBUG */


#endif /* _XTEV_SERIAL_DEBUG_H_ */
