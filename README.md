# ADI RPMsg-Lite tree

ADI downstream fork of
[NXP RPMsg-Lite](https://github.com/nxp-mcuxpresso/rpmsg-lite).

- Open pull requests against `adi-main`, which is based on the latest upstream
  release
- Prefix commits not intended for upstream with `[ADI]`

## CI/CD

This orphan branch holds ADI CI/CD workflows to keep them off the development
branches.

- `llm.yml`: LLM code review of a branch, tag, PR, or SHA

  ```sh
  gh workflow run llm.yml -R analogdevicesinc/rpmsg-lite -f ref=<pr-number>
  ```
