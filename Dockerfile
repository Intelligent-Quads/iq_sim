FROM gazebo:gzserver11-focal


ENV GAZEBO_MODEL_PATH=/iq_sim/models

RUN apt-get update -y && apt-get install -y git build-essential libssl-dev cmake libgazebo11-dev

RUN git clone https://github.com/khancyr/ardupilot_gazebo.git && cd ardupilot_gazebo && mkdir build && cd build && cmake .. && make -j4 && make install

WORKDIR /iq_sim

COPY . /iq_sim 

ENV WORLD=droneOnly

ENTRYPOINT gzserver --verbose worlds/${WORLD}.world