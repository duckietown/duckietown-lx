#!/bin/bash

# YOUR GITHUB TOKEN CREDENTIALS
GITHUB_USERNAME=YOUR_USERNAME_HERE
GITHUB_TOKEN=YOUR_TOKEN_HERE

## DO NOT MODIFY BELOW ##
git config --system url."https://$GITHUB_USERNAME:$GITHUB_TOKEN@github.com/".insteadOf "https://github.com/"
