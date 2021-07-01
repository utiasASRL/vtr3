FROM ubuntu:20.04

CMD ["/bin/bash"]

ARG DEBIAN_FRONTEND=noninteractive

ENV VTRROOT=/workspace
ENV VTRSRC=${VTRROOT}/vtr3
ENV VTRDEPS=${VTRROOT}/deps
ENV VTRDATA=${VTRROOT}/data
ENV VTRVENV=${VTRROOT}/venv
ENV VTRTEMP=${VTRROOT}/temp
RUN mkdir -p ${VTRROOT} ${VTRSRC} ${VTRDEPS} ${VTRDATA} ${VTRVENV} ${VTRTEMP}

RUN apt update

RUN apt install -q -y curl gnupg2 lsb-release build-essential cmake
RUN apt install -q -y libeigen3-dev