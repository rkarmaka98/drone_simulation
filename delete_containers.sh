#!/bin/bash

# stopping and deleting container
while true; do
    read -p "Do you want to stop and remove all containers? (y/n): " yn
    case $yn in
        [Yy]* ) 
            docker stop $(docker ps -q -l) && docker rm $(docker ps -q -l)
            break;;
        [Nn]* ) 
            exit;;
        * ) 
            echo "Please answer yes (y) or no (n).";;
    esac
done
