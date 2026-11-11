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

# Check that lines changed since <base> follow .clang-format
# Usage: check_clang_format <base>
check_clang_format () {
	local base=$1 out file git_cf

	# Prefer the git-clang-format matching the clang-format version
	git_cf="git-$RL_CI_CLANG_FORMAT"
	command -v "$git_cf" > /dev/null || git_cf=git-clang-format

	"$RL_CI_CLANG_FORMAT" --version
	out=$("$git_cf" --binary "$RL_CI_CLANG_FORMAT" --diff --extensions c,h \
		"$base" -- 2>&1)
	if [[ -z "$out" || "$out" == "no modified files to format" ||
	      "$out" == "clang-format did not modify any files" ]]; then
		return 0
	fi

	echo "$out"
	for file in $(echo "$out" | sed -n 's|^+++ b/||p'); do
		_annotate error "$file" 1 "clang-format: run 'git clang-format $base'"
	done
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
