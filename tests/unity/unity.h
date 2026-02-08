/* =========================================================================
    Unity - A Test Framework for C
    ThrowTheSwitch.org
    Copyright (c) 2007-26 Mike Karlesky, Mark VanderVoord, & Greg Williams
    SPDX-License-Identifier: MIT
    
    Minimal subset for SCARA project unit testing
========================================================================= */

#ifndef UNITY_H
#define UNITY_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Type definitions */
typedef unsigned int UNITY_UINT;
typedef int UNITY_INT;
typedef double UNITY_DOUBLE;

/* Unity state */
typedef struct {
    UNITY_UINT TestsFailed;
    UNITY_UINT TestsIgnored;
    UNITY_UINT TestsRun;
    const char* CurrentTestName;
    int CurrentTestFailed;
    int CurrentTestIgnored;
} UnityState;

extern UnityState Unity;

/* Core functions */
void UnityBegin(const char* filename);
int UnityEnd(void);
void UnityDefaultTestRun(void (*func)(void), const char* name, int line);

/* Assertion helpers */
void UnityFail(const char* msg, int line);
void UnityPass(void);
void UnityIgnore(const char* msg, int line);

/* Test macros */
#define TEST_ASSERT(condition) \
    do { if (!(condition)) { UnityFail(#condition, __LINE__); return; } } while(0)

#define TEST_ASSERT_TRUE(condition) TEST_ASSERT(condition)
#define TEST_ASSERT_FALSE(condition) TEST_ASSERT(!(condition))
#define TEST_ASSERT_NULL(pointer) TEST_ASSERT((pointer) == NULL)
#define TEST_ASSERT_NOT_NULL(pointer) TEST_ASSERT((pointer) != NULL)

#define TEST_ASSERT_EQUAL_INT(expected, actual) \
    do { if ((expected) != (actual)) { \
        printf("  Expected %d but was %d\n", (int)(expected), (int)(actual)); \
        UnityFail("Values not equal", __LINE__); return; \
    } } while(0)

#define TEST_ASSERT_EQUAL_UINT(expected, actual) \
    do { if ((expected) != (actual)) { \
        printf("  Expected %u but was %u\n", (unsigned)(expected), (unsigned)(actual)); \
        UnityFail("Values not equal", __LINE__); return; \
    } } while(0)

#define TEST_ASSERT_DOUBLE_WITHIN(delta, expected, actual) \
    do { if (fabs((expected) - (actual)) > (delta)) { \
        printf("  Expected %.6f +/- %.6f but was %.6f\n", (expected), (delta), (actual)); \
        UnityFail("Values not within tolerance", __LINE__); return; \
    } } while(0)

#define TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual) \
    TEST_ASSERT_DOUBLE_WITHIN((double)(delta), (double)(expected), (double)(actual))

#define TEST_ASSERT_EQUAL_DOUBLE(expected, actual) \
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, expected, actual)

#define TEST_ASSERT_EQUAL_STRING(expected, actual) \
    do { if (strcmp((expected), (actual)) != 0) { \
        printf("  Expected \"%s\" but was \"%s\"\n", (expected), (actual)); \
        UnityFail("Strings not equal", __LINE__); return; \
    } } while(0)

#define TEST_PASS() do { UnityPass(); return; } while(0)
#define TEST_FAIL() do { UnityFail("Failed", __LINE__); return; } while(0)
#define TEST_FAIL_MESSAGE(msg) do { UnityFail((msg), __LINE__); return; } while(0)
#define TEST_IGNORE() do { UnityIgnore("Ignored", __LINE__); return; } while(0)
#define TEST_IGNORE_MESSAGE(msg) do { UnityIgnore((msg), __LINE__); return; } while(0)

/* Runner macro */
#define RUN_TEST(func) UnityDefaultTestRun(func, #func, __LINE__)

#endif /* UNITY_H */
