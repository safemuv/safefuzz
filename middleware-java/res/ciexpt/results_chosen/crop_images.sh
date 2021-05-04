#!/bin/sh
find . -name "*.pdf" -exec pdfcrop -margins 10 '{}' '{}' ';'
