#!/bin/bash
PORT_NUM="$1"
ECAT_IFACE="$2"

LIFE_TEST=`rospack find life_test`
ECCOUNT="$LIFE_TEST/bin/eccount"
EC_MII="$LIFE_TEST/bin/ec_mii"
GRANT='pr2-grant'

if [[ -z "$PORT_NUM" ]] ; then 
    echo "Desc:"
    echo "  Disables auto-crossover on all port0 or or all port1"  
    echo "Usage : disable_xover <port_num> <ecat_iface>"
    echo "  ecat_iface : interface that EtherCAT device are on. Defaults to ecat0"
    echo "  port_num   : 0 for port0 or 1 for port1.  -1 to enable all port 0 and 1 "
    exit 1
fi

if [[ ! (-x "$ECCOUNT" ) ]] ; then
    echo "Cannot find eccount.  Expected to run it with '$ECCOUNT'"
    exit 1
fi

if [[ ! (-x "$EC_MII" ) ]] ; then
    echo "Cannot find ec_mii.  Expected to run it with '$EC_MII'"
    exit 1
fi


if [[ ( "$PORT_NUM" -ne 0 ) && ( "$PORT_NUM" -ne 1 ) && ( "$PORT_NUM" -ne -1 ) ]] ; then
    echo "Error, port number should be 0 or 1.  You said $PORT_NUM"
    exit 1
fi


if [[ -z "$ECAT_IFACE" ]] ; then
    ECAT_IFACE='ecat0'
    echo "No interface specified. Using $ECAT_IFACE"
fi


TMPFILE=`mktemp /tmp/disable_xover.XXXX`; RESULT=$?
if [[ "$RESULT" != "0" ]] ; then
    echo "ERROR: creating temp-file - $TMPFILE"
    exit 1
fi

function run() {
    CMD="$*"
    $CMD &> "$TMPFILE"
    RESULT=$?
    if [[ "$RESULT" != "0" ]]
    then 
	echo "ERROR running command : ${CMD}"
        cat  "$TMPFILE"
	exit 1
    fi
}

# use eccount to identify number of boards
# ecount returns value > 100 for errors
$GRANT $ECCOUNT -i $ECAT_IFACE
NUM_MCBS=$?
if [[ "$NUM_MCBS" -gt 100 ]] ; then 
    echo "eccount returned error code $NUM_MCBS"
    exit 1
fi

# first enable auto-crossover on all ports
echo "Enabling auto-crossover on all MCBs...."
for I in `seq $NUM_MCBS`; do
    run $GRANT $EC_MII -i $ECAT_IFACE -p $I -a 0 -xon
    run $GRANT $EC_MII -i $ECAT_IFACE -p $I -a 1 -xon
done

if [[ "$PORT_NUM" -ne -1 ]] ; then
    # next disable auto-crossover on selected port
    for I in `seq $NUM_MCBS`; do
        echo "Disabling auto-crossover on port $PORT_NUM of MCB $I"
        run $GRANT $EC_MII -i $ECAT_IFACE -p $I -a $PORT_NUM -xoff
    done
fi

# make sure disabling auto-crossover takes effect by force each link to restart autonegotiation 
#for I in `seq $NUM_MCBS`; do
#    echo "Resetarting auto-negotiation MCB $I"
#    run $EC_MII -i $ECAT_IFACE -p $I -a 0 -r
#done    

echo "Done"
