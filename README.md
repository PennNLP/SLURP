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
- Java SDK 1.6 (newer versions will almost certainly work but have
never been tested)
- sed
- Python 2.6 or 2.7 (not 3.x)

## Setup

- You need to download the Penn Pipeline by running `python
download.py`.  If you can't run that script on a machine with internet access,
download `http://www.seas.upenn.edu/~lignos/data/nlpipeline.zip` and unzip it
in the root of the repository.
- You will need to configure all the dependencies so
that the executable are on your PATH (i.e., java, sed, python).

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
