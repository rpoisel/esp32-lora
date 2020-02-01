#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

Import("env")

env['PROJECT_SRC_DIR'] = env['PROJECT_DIR'] + \
    os.sep + 'src' + os.sep + env['PIOENV'][env['PIOENV'].rfind('_')+1:]

# Dump build environment (for debug)
# print(env.Dump())
