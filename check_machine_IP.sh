#!/bin/bash

# Find your machine's IP address or container's IP address

hostname -I | awk '{print $1}'