#!/bin/sh
cd $SLURP_PATH
python -u pragbot_client.py > pragbot/log.txt 2> pragbot/errors.txt
