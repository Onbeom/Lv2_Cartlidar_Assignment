# 명령줄에 주어진 글자가 들어간 프로세스를 끔. 이 셸과 그 조상(부른 셸 · 터미널 · 도구)은 빼서 자기 자신을 죽이지 않음.
#   bash kill_procs.sh <신호 TERM|INT> <글자> [<글자> ...]
sig=$1; shift
anc=" $$ "; p=$$
while [ "$p" -gt 1 ] 2>/dev/null; do p=$(awk '{print $4}' /proc/$p/stat 2>/dev/null) || break; [ -n "$p" ] || break; anc="$anc$p "; done
for d in /proc/[0-9]*; do
  q=${d#/proc/}; case "$anc" in *" $q "*) continue;; esac
  c=$(tr '\0' ' ' < $d/cmdline 2>/dev/null) || continue
  for pat in "$@"; do case "$c" in *"$pat"*) kill -$sig $q 2>/dev/null; break;; esac; done
done
