SLURP
=====

*Situation Language Understanding Robot Platform*

SLURP is a system for converting natural language commands into
controls for a robot.

SLURP consists of a number of modules:

- Penn Pipeline: The UPenn natural language processing pipeline
- Semantics: Natural language understanding tools
- JR: Interface to the UMass Lowell Robotics Lab ATRV Jr. platform
- LTLBroom: Interface to the Cornell Autonomous Systems Lab LTLMoP platform

SLURP has been tested on OS X, Linux, and Windows.

Getting started
===============

## Downloading the code:
- If you use git, clone the repository URL above, i.e.,
`git clone https://github.com/PennNLP/SLURP.git`
- If you cannot use git, click "Downloads" above and download a .zip/.tar.gz.

## Dependencies:
- Java JRE 1.6 or 1.7
- Python 2.7 (not 3.x)
- Python numpy and Polygon2 (may be required because of LTLMoP)

## Setup

- You need to download the Penn Pipeline by running `python
download.py`.  If you can't run that script on a machine with internet access,
download `https://github.com/PennNLP/SUBTLEPipeline/archive/master.zip` and unzip
it in the root of the repository. You then need to decompress the parser model
located at `SUBTLEPipeline-master/models/wsjall.obj.gz` to `wsjall.obj` in the
same folder.
- You will need to configure all the dependencies so
that the executable are on your PATH (i.e., java, python).

## Testing the pipeline

To confirm that the pipeline is working, try the following:

- In one terminal, run `python pipelinehost.py`
- In another terminal, run `python test_pipelinehost.py` and enter
  any sentence you want and hit enter.

An example from a working system:

Terminal 1:

```text
$ python pipelinehost.py
pipelinehost: Waiting for connection...
pipelinehost: Connected to ('127.0.0.1', 51188)
Message: {u'text': u'This is a test.'}
Parsing...
Sending: '(S  (NP-SBJ-A (DT This)) (VP (VBZ is) (NP-PRD-A (DT a) (NN test)))(. .))'
```

Terminal 2:

```text
$ python test_pipelinehost.py
> This is a test.
Sending: '{"text": "This is a test."}\n'
Waiting for response...
(S  (NP-SBJ-A (DT This)) (VP (VBZ is) (NP-PRD-A (DT a) (NN test)))(. .))
```
Running In Conjunction with LTLMoP
----------------------------------
Clone the LTLMoP repository recursively with: git clone --recursive git://github.com/LTLMoP/LTLMoP.git

Checkout the LTLMoP gumbo branch via: git checkout gumbo

Compile the java code in LTLMoP/src/etc/jtlv via: sh build.sh

Checkout the frames (trouble checking out pragbot branch from LTLMoP) branch of SLURP via: 
* cd LTLMoP/src/etc/SLURP
* git fetch origin frames
* git checkout frames

Run:
* LTLMoP/src/etc/SLURP/pipelinehost.py
* LTLMoP/src/etc/SLURP/run_pragbot_slurp_server.sh
* Compile Pragbot code (see Pragbot repo)
* Pragbot/run_slurpserver.sh
* Pragbot/run_client.sh* 


