# shellcheck shell=bash
#
# Helpers for the ADI RPMsg-Lite CI/CD. Source this file from the root of an
# RPMsg-Lite checkout, e.g.:
#
#   source ci/build.sh
#   check_clang_format origin/adi-main
#   check_compile
#   check_analyzer
#   check_cppcheck origin/adi-main

# Platform headers to compile the core library against, skipped if missing
RL_CI_PLATFORMS=${RL_CI_PLATFORMS:-"adsp-sc5xx-sharc imx8mp_m7"}

# clang-format 16 is the closest match to the upstream formatting
RL_CI_CLANG_FORMAT=${RL_CI_CLANG_FORMAT:-clang-format-16}

# Newlib headers for the bare metal Arm target (Ubuntu libnewlib-dev)
RL_CI_NEWLIB_INCLUDE=${RL_CI_NEWLIB_INCLUDE:-/usr/include/newlib}

RL_CI_CORE_SOURCES=(
	lib/common/llist.c
	lib/rpmsg_lite/rpmsg_lite.c
	lib/rpmsg_lite/rpmsg_queue.c
	lib/rpmsg_lite/rpmsg_ns.c
	lib/virtio/virtqueue.c
	lib/rpmsg_lite/porting/environment/rpmsg_env_bm.c
)

RL_CI_CFLAGS=(
	--target=arm-none-eabi
	-mcpu=cortex-a5
	-isystem "$RL_CI_NEWLIB_INCLUDE"
	-std=c99
	-Wall
	-Wextra
	-Wno-unused-parameter
)

# Print a GitHub Actions annotation, or a plain message outside of GitHub
# Usage: _annotate <error|warning> <file> <line> <message>
_annotate () {
	local level=$1 file=$2 line=$3 msg=$4

	if [[ "$GITHUB_ACTIONS" == "true" ]]; then
		echo "::$level file=$file,line=$line::$msg"
	else
		echo "$level: $file:$line: $msg"
	fi
}

# Convert compiler and cppcheck output to annotations
# Usage: _annotate_diagnostics <error|warning> < output
_annotate_diagnostics () {
	local level=$1 file line msg

	while IFS=: read -r file line _ msg; do
		[[ "$line" =~ ^[0-9]+$ ]] || continue
		_annotate "$level" "$file" "$line" "${msg# }"
	done
}

# Print added lines as <file>:<line>
# Usage: _changed_lines <base>
_changed_lines () {
	git diff --unified=0 --no-color "$1" -- lib | awk '
		/^\+\+\+ / { file = substr($2, 3) }
		/^@@ / {
			split($3, a, ",")
			start = substr(a[1], 2)
			count = (a[2] == "") ? 1 : a[2]
			for (i = 0; i < count; i++)
				print file ":" start + i
		}'
}

# Report each change in a git-clang-format diff at its line, with the current
# and suggested lines and the changed characters highlighted. On GitHub, each
# change is also annotated at its line. Returns 2 if the input has no hunks.
# Usage: _clang_format_report <base> < diff
_clang_format_report () {
	local color=0

	if [[ -z "$NO_COLOR" && ( -t 1 || "$GITHUB_ACTIONS" == "true" ) ]]; then
		color=1
	fi

	awk -v base="$1" -v color="$color" -v gh="$GITHUB_ACTIONS" '
		# Escape GitHub workflow command data and properties
		function esc(s) {
			gsub(/%/, "%25", s); gsub(/\r/, "%0D", s); gsub(/\n/, "%0A", s)
			return s
		}
		function escp(s) {
			s = esc(s); gsub(/:/, "%3A", s); gsub(/,/, "%2C", s)
			return s
		}
		# Highlight s[from..to] with on/off, or with «» and visible
		# whitespace if on is empty. An empty range marks a position.
		function hl(s, from, to, on, off,    mid) {
			mid = substr(s, from, to - from + 1)
			if (on == "") {
				gsub(/\t/, "→", mid); gsub(/ /, "·", mid)
				on = "«"; off = "»"
			}
			return substr(s, 1, from - 1) on mid off substr(s, to + 1)
		}
		# Set P to the common prefix length of o and n, and OE/NE to the
		# end of the differing part in o/n
		function span(o, n,    lo, ln, s) {
			lo = length(o); ln = length(n)
			for (P = 0; P < lo && P < ln; P++)
				if (substr(o, P + 1, 1) != substr(n, P + 1, 1))
					break
			for (s = 0; s < lo - P && s < ln - P; s++)
				if (substr(o, lo - s, 1) != substr(n, ln - s, 1))
					break
			OE = lo - s; NE = ln - s
		}
		function norm(s) {
			gsub(/[ \t]+/, "", s)
			return s
		}
		# Append a removed, added or changed line to out, and the
		# suggested line to msg
		function del(i) {
			out = out sprintf("%6d ", start + i - 1) red "-" d[i] rst "\n"
		}
		function add(j) {
			out = out "       " grn "+" a[j] rst "\n"
			msg = msg "\n+" a[j]
		}
		function pair(i, j) {
			span(d[i], a[j])
			out = out sprintf("%6d ", start + i - 1) red "-" \
			      hl(d[i], P + 1, OE, ron, roff) rst "\n"
			out = out "       " grn "+" hl(a[j], P + 1, NE, gon, goff) rst "\n"
			msg = msg "\n+" hl(a[j], P + 1, NE)
		}
		function flush(    i, j, l, e, L) {
			if (nd + na == 0)
				return
			issues++
			# A pure insertion is reported at the line before it
			l = nd ? start : (start > 1 ? start - 1 : 1)
			e = nd ? start + nd - 1 : l
			out = sprintf("%s:%d%s: clang-format\n", file, l, e > l ? "-" e : "")
			msg = na ? "Suggested:" : "Suggested: remove the line(s)"
			if (nd == na) {
				for (i = 1; i <= nd; i++)
					pair(i, i)
			} else {
				# Lines were split or joined: pair up the lines that only
				# differ in whitespace (LCS), show the others whole
				for (i = 1; i <= nd; i++)
					dn[i] = norm(d[i])
				for (j = 1; j <= na; j++)
					an[j] = norm(a[j])
				for (i = nd + 1; i >= 1; i--)
					for (j = na + 1; j >= 1; j--)
						if (i > nd || j > na)
							L[i, j] = 0
						else if (dn[i] == an[j])
							L[i, j] = L[i + 1, j + 1] + 1
						else if (L[i + 1, j] >= L[i, j + 1])
							L[i, j] = L[i + 1, j]
						else
							L[i, j] = L[i, j + 1]
				for (i = j = 1; i <= nd || j <= na; )
					if (i <= nd && j <= na && dn[i] == an[j])
						pair(i++, j++)
					else if (j > na || (i <= nd && L[i + 1, j] >= L[i, j + 1]))
						del(i++)
					else
						add(j++)
			}
			if (gh == "true")
				printf "::error file=%s,line=%d,endLine=%d,title=clang-format::%s\n",
				       escp(file), l, e, esc(msg)
			printf "%s", out
			nd = na = 0
		}
		BEGIN {
			hdr = 1
			if (color) {
				red = "\033[31m"; grn = "\033[32m"; rst = "\033[0m"
				ron = "\033[97;41m"; roff = "\033[0;31m"
				gon = "\033[97;42m"; goff = "\033[0;32m"
			}
		}
		/^diff --git / { flush(); hdr = 1; next }
		# A removed "--i;" shows as "---i;", so only parse --- and +++ in headers
		hdr && /^\+\+\+ b\// { file = substr($0, 7); next }
		/^@@ / {
			flush(); hdr = 0
			split($2, r, ",")
			old = substr(r[1], 2) + 0
			next
		}
		hdr || /^\\/ { next }
		/^-/ {
			if (nd + na == 0)
				start = old
			d[++nd] = substr($0, 2); old++
			next
		}
		/^\+/ {
			if (nd + na == 0)
				start = old
			a[++na] = substr($0, 2)
			next
		}
		{ flush(); old++ }
		END {
			flush()
			if (!issues)
				exit 2
			printf "%d clang-format issue(s), fix with: git clang-format %s\n",
			       issues, base
		}'
}

_include_flags () {
	local platform=$1

	echo "-Ilib/include -Ilib/include/platform/$platform" \
	     "-Ilib/include/environment/bm -Itemplate_application"
}

_platforms () {
	local platform

	for platform in $RL_CI_PLATFORMS; do
		[[ -d "lib/include/platform/$platform" ]] && echo "$platform"
	done
}

# Check that lines changed since <base> follow .clang-format. Each change is
# reported at its line with the suggested fix, followed by the full diff.
# Usage: check_clang_format <base>
check_clang_format () {
	local base=$1 out git_cf

	# Prefer the git-clang-format matching the clang-format version
	git_cf="git-$RL_CI_CLANG_FORMAT"
	command -v "$git_cf" > /dev/null || git_cf=git-clang-format

	"$RL_CI_CLANG_FORMAT" --version
	# git-clang-format prints a porcelain git diff, pin the format it is
	# parsed with
	out=$(GIT_CONFIG_COUNT=3 \
		GIT_CONFIG_KEY_0=color.diff GIT_CONFIG_VALUE_0=never \
		GIT_CONFIG_KEY_1=diff.noprefix GIT_CONFIG_VALUE_1=false \
		GIT_CONFIG_KEY_2=diff.mnemonicPrefix GIT_CONFIG_VALUE_2=false \
		"$git_cf" --binary "$RL_CI_CLANG_FORMAT" --diff --extensions c,h \
		"$base" -- 2>&1) || true
	if [[ -z "$out" || "$out" == "no modified files to format" ||
	      "$out" == "clang-format did not modify any files" ]]; then
		return 0
	fi

	# Not a diff, e.g. an unknown <base>
	if ! echo "$out" | _clang_format_report "$base"; then
		echo "$out"
		return 1
	fi

	if [[ "$GITHUB_ACTIONS" == "true" ]]; then
		echo "::group::clang-format diff"
		echo "$out"
		echo "::endgroup::"
	else
		printf '\nclang-format diff:\n%s\n' "$out"
	fi
	return 1
}

# Compile the core library with Clang for each available platform
check_compile () {
	local platform src out fail=0
	local -a inc

	clang --version | head -1
	for platform in $(_platforms); do
		read -r -a inc <<< "$(_include_flags "$platform")"
		for src in "${RL_CI_CORE_SOURCES[@]}"; do
			echo "clang ($platform): $src"
			out=$(clang "${RL_CI_CFLAGS[@]}" -Werror "${inc[@]}" \
				-c "$src" -o /dev/null 2>&1) || fail=1
			[[ -n "$out" ]] && echo "$out" && \
				echo "$out" | grep -E ': (error|warning):' | _annotate_diagnostics error
		done
	done
	return $fail
}

# Run the Clang static analyzer on the core library for each available platform
check_analyzer () {
	local platform src out fail=0
	local -a inc

	for platform in $(_platforms); do
		read -r -a inc <<< "$(_include_flags "$platform")"
		for src in "${RL_CI_CORE_SOURCES[@]}"; do
			echo "clang --analyze ($platform): $src"
			out=$(clang --analyze "${RL_CI_CFLAGS[@]}" "${inc[@]}" \
				"$src" -o /dev/null 2>&1)
			if echo "$out" | grep -q -E ': (error|warning):'; then
				fail=1
				echo "$out"
				echo "$out" | grep -E ': (error|warning):' | _annotate_diagnostics error
			fi
		done
	done
	return $fail
}

# Run cppcheck on C files changed since <base> and annotate findings on changed
# lines. Findings are warnings since the CCES and SDK headers are not available.
# Usage: check_cppcheck <base>
check_cppcheck () {
	local base=$1 file platform out line
	local -a inc defs files

	mapfile -t files < <(git diff --name-only --diff-filter=AM "$base" -- 'lib/*.c')
	# Header changes affect the core library
	if git diff --name-only --diff-filter=AM "$base" -- 'lib/*.h' | grep -q .; then
		files+=("${RL_CI_CORE_SOURCES[@]}")
	fi
	mapfile -t files < <(printf '%s\n' "${files[@]}" | sort -u | grep .)

	[[ ${#files[@]} -eq 0 ]] && echo "No C files changed" && return 0

	cppcheck --version
	_changed_lines "$base" > /tmp/rl-changed-lines.txt
	for file in "${files[@]}"; do
		platform=$(_platforms | head -1)
		defs=(-D__GNUC__)
		if [[ "$file" =~ porting/platform/([^/]+)/ ]]; then
			platform=${BASH_REMATCH[1]}
		fi
		if [[ "$platform" == adsp-sc5xx-sharc && "$file" == *"/$platform/"* ]]; then
			defs=(-D__CCESVERSION__ -D__ADSPSHARC__ -D__ADI_HAS_TRU__)
		fi
		read -r -a inc <<< "$(_include_flags "$platform")"

		echo "cppcheck ($platform): $file"
		out=$(cppcheck -q --std=c99 --enable=warning,style,performance,portability \
			--inline-suppr --suppress=missingInclude --suppress=missingIncludeSystem \
			--template='{file}:{line}:{column}: {severity}: {message} [{id}]' \
			"${defs[@]}" "${inc[@]}" "$file" 2>&1)
		echo "$out" | grep -E '^[^ ]+:[0-9]+:[0-9]+: ' | while IFS= read -r line; do
			if grep -q -x -F "$(echo "$line" | cut -d: -f1,2)" /tmp/rl-changed-lines.txt; then
				echo "$line"
				echo "$line" | _annotate_diagnostics warning
			fi
		done
	done
	return 0
}
