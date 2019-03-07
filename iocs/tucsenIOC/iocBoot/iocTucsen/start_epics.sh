export EPICS_CA_SERVER_PORT=5077
medm -x -macro "P=13TUCSEN:,R=cam1:" ../../../../tucsenApp/op/adl/Tucsen.adl &
../../bin/linux-x86_64/tucsenApp st.cmd
