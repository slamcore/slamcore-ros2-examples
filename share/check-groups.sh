#!/usr/bin/env bash
user=$(whoami)

mandatory_groups="video"
for group in $mandatory_groups
do
    getent group $group | grep -q "\b$user\b"
    in_group=$?
    if [[ $in_group != "0" ]]
    then
        echo
        echo "Host user \"$user\" is not in the \"$group\" group."
        echo "Please add the user to the group and retry"
        echo
        exit 1
    fi
done
