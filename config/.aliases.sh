#!/bin/bash

# Custom aliases for container internal user's shell

alias zshconfig="code ~/.zshrc"
alias ohmyzsh="code ~/.oh-my-zsh"
alias p10k="code ~/.p10k.zsh"
alias update="sudo apt-get update && sudo apt full-upgrade -y"
alias ls="lsd --group-dirs=first -S"
alias ll="ls -l --total-size -h"
alias la="ll -a"
alias lt="ls --tree --depth=1 "
alias clc='clear'
alias py2='source ~/venvs/py27/bin/activate'
alias py3='source ~/venvs/py310/bin/activate'
alias pip='python -m pip'
alias cm='cd ~/catkin_ws && catkin_make && source devel/setup.zsh'