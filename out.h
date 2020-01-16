typedef struct { const char *name; const char *mime; int size; const unsigned char *data; } stFile;
#define FS_FILES	0


const volatile stFile fsFiles[0] = { };