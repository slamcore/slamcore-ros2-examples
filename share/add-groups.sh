#!/usr/bin/env bash
ADDITIONAL_GROUPS=$1
for gid in $(echo $ADDITIONAL_GROUPS | tr "," " ")
do
    group_name=$(cat /dev/urandom | tr -cd 'a-f0-9' | head -c 5)
    groupadd -g $gid $group_name || true
done
