# RobotLab

A robot simulation using Python and PyBullet.

## Requirements

Python and Git.

## Downloading

```bash
git clone https://github.com/BoyanK1/C1---Robot-Arcade.git
cd C1---Robot-Arcade
```

Alternatively, download the repository as a ZIP and extract it.
Open a terminal in the extracted project folder.

## Virtual Env.

Windows:

```powershell
py -m venv .venv
.venv\Scripts\Activate.ps1
```

macOS/Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

## Install dependencies

With the virtual environment activated:

```bash
python -m pip install -r requirements.txt
```

## Start-up

```bash
python main.py
```

After starting up the program a Panda robot will appear as well as 3 cubes and 3 checkpoints. The robot will then start to move towards the first cube slowly. After both fingers touch the cube, a fixed constraint attaches it to the gripper. The robot carries it to the matching checkpoint, lowers it and removes the constraint to release it. It repeats this for all three cubes and prints the score. At the end, the Panda stays above the last checkpoint and the window remains open. To exit the programme, press Ctrl+C in the terminal.

## Tests

```bash
python -m unittest discover -s tests -v
```

The tests run without a GUI. They check that a cube cannot attach across an air gap, both fingers touch each cube before attachment, and the full round scores 3/3.
