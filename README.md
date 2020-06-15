# aiomi_arduino
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/c4191d25c02044aca5fe01e653d907b5)](https://app.codacy.com/gh/TEAM-A-aiomi/aiomi_arduino?utm_source=github.com&utm_medium=referral&utm_content=TEAM-A-aiomi/aiomi_arduino&utm_campaign=Badge_Grade_Settings)
# Travis Badge: [![Build Status](https://travis-ci.org/TEAM-A-aiomi/aiomi_arduino.svg?branch=master)](https://travis-ci.org/TEAM-A-aiomi/aiomi_arduino)

bare_metal_code_wrapper.c tests our equivalent analogRead, digitalRead, and digitalWrite wrappers (analogWrite blocks on the COM0B0 setup line, so we can get it to blink, but then not update after that)

bare_metal_code.c has our program that does not fully work because of various issues (logged in issues)

arduino_version_complete has a complete and working program working in the Arduino language

All other files are Arduino and bare metal C version that were backed up as separate development versions for clarity in what they contain (we know that a real project would not have these and would instead use git commit comments and external tools to keep track of features, but this seemed good for us)
