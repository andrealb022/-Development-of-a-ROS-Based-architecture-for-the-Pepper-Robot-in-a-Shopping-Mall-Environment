#!/bin/bash

BOT_DIR="/home/paolo/Scrivania/progetto/src/bot"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
