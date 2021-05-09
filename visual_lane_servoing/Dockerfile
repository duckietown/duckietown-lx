FROM duckietown/challenge-aido_lf-baseline-duckietown:daffy-amd64

RUN pip3 install -U "pip>=20.2"
COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN  pip3 install --use-feature=2020-resolver -r .requirements.txt

RUN pip list

RUN mkdir /code/solution

COPY solution /code/solution
COPY launchers /code

CMD ["bash", "/code/submit.sh"]