FROM python:3.7

WORKDIR /duckietown_aido_ros_bridge
COPY requirements.txt .
ARG PIP_INDEX_URL="https://pypi.org/simple/"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
RUN echo PIP_INDEX_URL=${PIP_INDEX_URL}

RUN python3 -m pip install -r requirements.txt

COPY . .

RUN find .

ENV DISABLE_CONTRACTS=1

RUN pipdeptree
RUN python setup.py install --no-deps
# run it once to see everything OK
RUN dt-pc-demo --help
CMD ["dt-pc-demo"]
