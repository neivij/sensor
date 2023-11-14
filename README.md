# Sensor node bus firmware

Code Composer Studio project for BNO055 sensor node firmware. The program samples BNO055 sensor and sends over an asynchronous bus using UART and my special protocol.

# Usage

The repo contains 4 branches:
* main - the main branch for sensor code
* master-node - the branch for "master" node which will invoke sampling with fixed intervals. Acts as sensor with ID = 0
* test-thr - the branch for sensor code to test protocol throughput
* m-n-test-thr - the branch for master sensor node to test protocol throughput
