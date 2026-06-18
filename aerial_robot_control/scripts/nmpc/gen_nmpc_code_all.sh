#!/bin/bash

# ===== avoid jobserver warning when "catkin build", given by ChatGPT o3
set -e
unset MAKEFLAGS
# =====

MODELS=(
    # MHEVelDynIMU
    # MHEWrenchEstAccMom
    # MHEWrenchEstIMUAct
    # MHEWrenchEstMomentum
    # NMPCTiltQdNoServo
    # NMPCTiltQdNoServoAcCost
    # NMPCTiltQdServo
    # NMPCTiltQdServoDiff
    # NMPCTiltQdServoDist
    # NMPCTiltQdServoDragDist
    # NMPCTiltQdServoImpedance
    # NMPCTiltQdServoOldCost
    # NMPCTiltQdServoThrust
    # NMPCTiltQdServoThrustDist
    # NMPCTiltQdServoThrustDrag
    # NMPCTiltQdServoThrustImpedance
    # NMPCTiltQdServoWCogEndDist
    # NMPCTiltQdThrust
)

for model in "${MODELS[@]}"
do
    echo "Generating NMPC code for model: $model"
    python3 gen_nmpc_code.py -m "$model"
done
