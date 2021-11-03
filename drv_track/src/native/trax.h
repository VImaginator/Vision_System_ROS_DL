/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*- */

#ifndef TRAX_H
#define TRAX_H

#include <stdio.h>
#include <fcntl.h>

#ifndef __TRAX_EXPORT
#if defined(_MSC_VER)
    #define __TRAX_EXPORT __declspec(dllexport)
#elif defined(_GCC)
    #define __TRAX_EXPORT __attribute__((visibility("default")))
#else
    #define __TRAX_EXPORT
#endif
#endif

#if defined(__OS2__) || defined(__WINDOWS__) || defined(WIN32) || defined(WIN64) || defined(_MSC_VER)
    #define TRAX_NO_LOG (~0)
#else
    #define TRAX_NO_LOG -1
#endif

#define TRAX_VERSION 1

#define TRAX_ERROR -1
#define TRAX_HELLO 0
#define TRAX_INITIALIZE 1
#define TRAX_FRAME 2
#define TRAX_QUIT 3
#define TRAX_STATUS 4
#define TRAX_STATE 4

#define TRAX_IMAGE_PATH 0
#define TRAX_IMAGE_URL 1 // Not implemented yet!
#define TRAX_IMAGE_DATA 2 // Not implemented yet!

#define TRAX_REGION_RECTANGLE 1
#define TRAX_REGION_POLYGON 2
#define TRAX_REGION_MASK 3 // Not implemented yet!
#define TRAX_REGION_SPECIAL 0

#define TRAX_FLAG_VALID 1
#define TRAX_FLAG_SERVER 2
#define TRAX_FLAG_TERMINATED 4

#define TRAX_PARAMETER_VERSION 0
#define TRAX_PARAMETER_CLIENT 1
#define TRAX_PARAMETER_SOCKET 2
#define TRAX_PARAMETER_REGION 3

#define TRAX_LOCALHOST "127.0.0.1"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * A trax image data structure. At the moment only the path format is supported
 * so a lot of fields are empty.
 * Use trax_image_* functions to access the data.
**/
typedef struct trax_image {
    short type;
    int width;
    int height;
    char* data;
} trax_image;

/**
 * A placeholder for region structure. Use the trax_region_* functions to manipulate
 * the data.
**/
typedef void trax_region;

/**
 * A placeholder for properties structure. Use the trax_properties_* functions to manipulate
 * the data.
**/
typedef struct trax_properties trax_properties;

/**
 * Some basic configuration data used to set up the server.
**/
typedef struct trax_configuration {
        int format_region;
        int format_image;
} trax_configuration;

/**
 * Core object of the protocol. Do not manipulate it directly.
**/
typedef struct trax_handle {
    int flags;
    int version;
    void* stream;
    trax_properties* properties;
    FILE* log;
    trax_configuration config;
} trax_handle;

typedef void(*trax_enumerator)(const char *key, const char *value, const void *obj);

__TRAX_EXPORT const char* trax_version();

/**
 * Setups the protocol state object for the client and returns a handle object.
**/
__TRAX_EXPORT trax_handle* trax_client_setup_file(int input, int output, FILE* log);

/**
 * Setups the protocol state object for the client and returns a handle object.
**/
__TRAX_EXPORT trax_handle* trax_client_setup_socket(int server, FILE* log);

/**
 * Waits for a valid protocol message from the server.
**/
__TRAX_EXPORT int trax_client_wait(trax_handle* client, trax_region** region, trax_properties* properties);

/**
 * Sends an initialize message.
**/
__TRAX_EXPORT void trax_client_initialize(trax_handle* client, trax_image* image, trax_region* region, trax_properties* properties);

/**
 * Sends a frame message.
**/
__TRAX_EXPORT void trax_client_frame(trax_handle* client, trax_image* image, trax_properties* properties);

/**
 * Setups the prot