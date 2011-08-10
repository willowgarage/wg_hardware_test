#!/bin/bash

ECAT_IFACE=ecat0

CROSSOVER_MODE=`rosparam get crossover_mode`
if [[ "$?" != "0" ]] ; then
    echo "ERROR : rosparam get crossover_mode"
    exit 1
fi

DISABLE_XOVER=`rospack find life_test`/scripts/disable_xover.sh

function run() {
    CMD="$*"
    RESULT=$?
    if [[ "$RESULT" != "0" ]]
    then 
	echo "ERROR running command : ${CMD}"
	exit 1
    fi
}

case "$CROSSOVER_MODE" in 
    "disable_port_0")
        run $DISABLE_XOVER 0 $ECAT_IFACE
        echo "Disable auto-xover on port 0"
        ;;
    "disable_port_1")
        run $DISABLE_XOVER 1 $ECAT_IFACE
        echo "Disable auto-xover on port 1"
        ;;
    *)
        echo "ERROR : invalid crossover_mode : $CROSSOVER_MODE"
esac

# Run pr2_etherCAT using pr2_grant
exec pr2-grant `rospack find pr2_etherCAT`/pr2_etherCAT $@
