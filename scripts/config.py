#!/usr/bin/env python
# The post processor, global variable used in every service server
global pp
pp = None

# Error message if a service is called prior to initializing a post-processor
global pp_not_init
pp_not_init = "No post processor initialized.\nCall prog_start service first."
