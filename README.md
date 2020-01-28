# LoRa-Telemetry
Radio-Based telemetry system for UPCSP's HABs

Radio-based comms system designed to work with UPCSP's Near Space Probe, in the context of the High Altitude Ballooning mission. The code is responsible for obtaining data from the probe such as: Altitude, Coordinates, number of available satellites and Temperature. Emitter is designed to be executed on the Probe and Receiver on the ground.

Future versions of the code will include automatic uploading of the coordinates to a DB in AWS, to perform real-time landing predictions. This is being coded in Python in a RasPi Zero.
