#!/bin/bash

git push

git subtree push --prefix=apa_services apa_services master

git subtree push --prefix=apa_runtime apa_runtime public

git subtree push --prefix=apa_common apa_common main
