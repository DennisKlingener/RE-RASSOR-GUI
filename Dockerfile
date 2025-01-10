# Use the 22.04 version of ubuntu for our base image.
FROM ubuntu:22.04

# This sets the working directory for the container to "~/opt". This is where all the subsequent command will be ran.
WORKDIR /opt

# Install git so that we can clone the needed repos.
RUN apt-get update && apt-get install -y git

# Now we can go ahead and clone the capabiltites repo.
#RUN git clone https://github.com/FlaSpaceInst/2024-RE-RASSOR-Capabilities.git

# ENTRYPOINT (This will be helpful later)

