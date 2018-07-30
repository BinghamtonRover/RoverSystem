# Binghamton University Mars Rover System Overview

This document describes the rover software and system architecture at a broad scale.

## Table of Contents

1. [Overview](#overview)
2. [Base Station](#base-station)
3. [Rover](#rover)

## Overview

The rover system is comprised of two main components: the base station (abbreviated BS), and the rover (abbreviated R). The base station controls the rover's actions and displays sensor and diagnostic information. The two components communicate via a network protocol described in this document. Since both components are on a local network, they have hardcoded IP addresses. Although this implies that each component knows the location of the other, it is the base station's responsibility to initiate connection to the rover, and to recover it in the event of connection loss. Since the base station is used to query rover state and to execute remote commands, it is fitting to describe the rover as a server and the base station as a client; however, since there is only one base station, there is no need to allow for multiple connections to the rover.

## Base Station

The base station is the control center for the rover. It is used to control the rover in manual mode, to view the rover's camera feeds, to view rover sensor data and logs, and to command the rover to complete autonomous tasks.

## Rover