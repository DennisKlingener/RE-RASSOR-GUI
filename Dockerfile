# Use the 22.04 version of ubuntu for our base image.
FROM ubuntu:22.04

# This sets the working directory for the container to "~/opt". This is where all the subsequent commands will be run.
WORKDIR /opt

# Copy the submodules over to the container.
COPY ./externs/. /opt/

# ENTRYPOINT (This will be helpful later)