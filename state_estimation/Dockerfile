ARG DOCKER_REGISTRY=docker.io
FROM ${DOCKER_REGISTRY}/duckietown/challenge-aido_lf-baseline-duckietown-ml:daffy-amd64

COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN python3 -m pip install  -r .requirements.txt

RUN python3 -m pip list

RUN mkdir /code/solution

COPY solution /code/solution
COPY launchers /code

CMD ["bash", "/code/submit.sh"]
