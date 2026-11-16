# Sourced in a pane (bash or zsh):  . ./history_prefill.sh "<cmd>" "<cmd>" ...
# Puts the commands into this shell's history, so arrow-up offers them right away; the LAST one
# comes first. With atuin active (`atuin init` in .bashrc / .zshrc) arrow-up shows atuin's history
# instead, which `history -s` / `print -s` never reach, so each command also goes into atuin's
# history of this shell session (ATUIN_FILTER_MODE=session in pre_window). Note that atuin keeps
# them: they also show up in atuin searches outside the session.
for _hp_c in "$@"; do
  if [ -n "$ZSH_VERSION" ]; then print -s -- "$_hp_c"; else history -s "$_hp_c"; fi
  if [ -n "$ATUIN_SESSION" ] && command -v atuin >/dev/null 2>&1; then
    # what atuin's own shell hook does; `history end` stores the entry, then lingers: background it
    _hp_i=$(atuin history start -- "$_hp_c" 2>/dev/null) && [ -n "$_hp_i" ] &&
      (atuin history end --exit 0 --duration 0 -- "$_hp_i" >/dev/null 2>&1 &)
  fi
done
unset _hp_c _hp_i
