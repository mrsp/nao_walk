#!/bin/bash
rosservice call /nao_speak "thank you jaco."
rosservice call /nao_speak "Now is time to rest."
rosservice call /nao_behaviours/crouch
