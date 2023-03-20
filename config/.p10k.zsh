zstyle ':omz:update' mode auto
ENABLE_CORRECTION='true'
HIST_STAMPS='dd.mm.yyyy'
typeset -g POWERLEVEL9K_MODE=nerdfont-complete
typeset -g POWERLEVEL9K_OS_ICON_BACKGROUND=208
typeset -g POWERLEVEL9K_VIRTUALENV_FOREGROUND=220
typeset -g POWERLEVEL9K_VIRTUALENV_BACKGROUND=28
typeset -g POWERLEVEL9K_VIRTUALENV_SHOW_WITH_PYENV=false
# pip zsh completion start
function _pip_completion {
  local words cword
  read -Ac words
  read -cn cword
  reply=( $( COMP_WORDS="$words[*]" \
             COMP_CWORD=$(( cword-1 )) \
             PIP_AUTO_COMPLETE=1 $words[1] ) )
}
compctl -K _pip_completion pip
# pip zsh completion end
