#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#define APPEND_TO_GPT_QUESTION_FILE_ENABLE    1





#define DEBUG_ENABLE      1

#define DEBUG_FILE_LINE_ENABLE    1


#define sys_log_plain(fmt, ...) \
printf("[INFO]: " fmt "\n", ##__VA_ARGS__)


#define warn_log_plain(fmt, ...) \
printf("\033[33m[WARNING]: " fmt "\033[0m\n", ##__VA_ARGS__)


/* 这个在linux终端中打印出的error是红色，更加醒目 */
#define err_log_plain(fmt, ...) \
printf("\033[31m[ERROR]: " fmt "\033[0m\n", ##__VA_ARGS__)


#if DEBUG_ENABLE 

    #include <stdio.h>

    #if DEBUG_FILE_LINE_ENABLE

    #define sys_log(fmt, ...) \
        printf("[INFO] %s:%u: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)

    /* 这个在linux终端中打印出的error是红色，更加醒目 */
    #define err_log(fmt, ...) \
        printf("\033[31m[ERROR] %s:%u: " fmt "\033[0m\n", __FILE__, __LINE__, ##__VA_ARGS__)

    #define warn_log(fmt, ...) \
        printf("\033[33m[WARNING] %s:%u: " fmt "\033[0m\n", __FILE__, __LINE__, ##__VA_ARGS__)


    #else

    #define sys_log(fmt, ...) \
        printf("[INFO]: " fmt "\n", ##__VA_ARGS__)


    #define warn_log(fmt, ...) \
        printf("\033[33m[WARNING]: " fmt "\033[0m\n", ##__VA_ARGS__)


    /* 这个在linux终端中打印出的error是红色，更加醒目 */
    #define err_log(fmt, ...) \
        printf("\033[31m[ERROR]: " fmt "\033[0m\n", ##__VA_ARGS__)

    #endif  // DEBUG_FILE_LINE_ENABLE

#else 

#define sys_log
#define err_log
#define warn_log

#endif // DEBUG_ENABLE



#define PRINT_TIME() do { \
    time_t now = time(NULL); \
    struct tm *tm_info = localtime(&now); \
    char time_str[20]; \
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm_info); \
    sys_log("%s", time_str); \
} while (0)




#endif // USER_CONFIG_H
