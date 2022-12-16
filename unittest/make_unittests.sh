#!/bin/bash

start_time=$(date +%s)
now=$(date +"%m_%d_%Y")
time=$(date +"%H:%M:%S")
log_file=../log_file.txt

printf "========================================================\n\n" #> $log_file
printf "Starting UNITTESTS - Time: %s\n\n" $time #>> $log_file

source_dir_list=(
    # clock_driver_rp2040
    gpio_driver_rp2040
    irq_interface_rp2040
    uart_driver_rp2040
)

for i in "${!source_dir_list[@]}"; do

        source_dir=${source_dir_list[$i]}
    time=$(date +"%H:%M:%S")

        printf "\n\n%s\n" "---------------------------------------------------------------" #>> $log_file
        printf " - TIME: $time\n" #>> $log_file
        printf " - SOURCE: $source_dir\n" #>> $log_file

        # check if source is available
        if ls | grep $source_dir > /dev/null; then
                printf "%s\n" " - AVAILABLE" #>> $log_file
        else
                printf "%s\n" " - NOT AVAILABLE !!!" #>> $log_file
                continue
        fi

        # check if src-dir is empty to prevent rsync from delete data within backup-dir
        if [ -z "$(ls -A $source_dir)" ]; then
                printf "%s\n" "- EMPTY !!" #>> $log_file
                continue
        else
                printf "%s\n" " - NOT EMPTY" #>> $log_file
        fi

        printf "%s\n" " - STARTING UNITTEST:" #>> $log_file

    cd $source_dir
    if make clean all run; then
        printf "%s\n" "- SUCCESS";
    else
        printf "%s\n" "- FAILED !!!";
        exit 1;
    fi

    printf "%s\n" " - CLEANING UP"
    make clean

    printf "\n\n"
    cd ..
done
