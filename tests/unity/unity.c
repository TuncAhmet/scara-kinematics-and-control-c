/* =========================================================================
    Unity - A Test Framework for C
    ThrowTheSwitch.org
    Copyright (c) 2007-26 Mike Karlesky, Mark VanderVoord, & Greg Williams
    SPDX-License-Identifier: MIT
    
    Minimal implementation for SCARA project unit testing
========================================================================= */

#include "unity.h"

UnityState Unity;

void UnityBegin(const char* filename) {
    Unity.TestsFailed = 0;
    Unity.TestsIgnored = 0;
    Unity.TestsRun = 0;
    Unity.CurrentTestName = NULL;
    Unity.CurrentTestFailed = 0;
    Unity.CurrentTestIgnored = 0;
    
    printf("\n");
    printf("========================================\n");
    printf(" Unity Test Framework\n");
    printf(" File: %s\n", filename);
    printf("========================================\n\n");
}

int UnityEnd(void) {
    printf("\n========================================\n");
    printf(" Tests Run: %u\n", Unity.TestsRun);
    printf(" Passed:    %u\n", Unity.TestsRun - Unity.TestsFailed - Unity.TestsIgnored);
    printf(" Failed:    %u\n", Unity.TestsFailed);
    printf(" Ignored:   %u\n", Unity.TestsIgnored);
    printf("========================================\n");
    
    if (Unity.TestsFailed > 0) {
        printf(" FAIL\n\n");
        return 1;
    }
    printf(" OK\n\n");
    return 0;
}

void UnityDefaultTestRun(void (*func)(void), const char* name, int line) {
    (void)line;
    
    Unity.CurrentTestName = name;
    Unity.CurrentTestFailed = 0;
    Unity.CurrentTestIgnored = 0;
    Unity.TestsRun++;
    
    printf("Test: %s ... ", name);
    fflush(stdout);
    
    setUp();     /* Call setUp before each test */
    func();
    tearDown();  /* Call tearDown after each test */
    
    if (Unity.CurrentTestFailed) {
        printf("FAIL\n");
    } else if (Unity.CurrentTestIgnored) {
        printf("IGNORED\n");
    } else {
        printf("PASS\n");
    }
}

void UnityFail(const char* msg, int line) {
    Unity.CurrentTestFailed = 1;
    Unity.TestsFailed++;
    printf("\n  [FAIL] Line %d: %s\n  ", line, msg);
}

void UnityPass(void) {
    /* Nothing to do, test passed */
}

void UnityIgnore(const char* msg, int line) {
    Unity.CurrentTestIgnored = 1;
    Unity.TestsIgnored++;
    printf("\n  [IGNORED] Line %d: %s\n  ", line, msg);
}

/* Optional setUp/tearDown stubs if not defined */
__attribute__((weak)) void setUp(void) {}
__attribute__((weak)) void tearDown(void) {}
