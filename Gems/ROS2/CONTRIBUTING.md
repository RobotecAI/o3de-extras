
### Code of conduct

Please refer to and respect the [O3DE Code of Conduct](https://www.o3de.org/docs/contributing/code-of-conduct/).

### Authorship and Developer Certificate of Origin (DCO)

When creating a pull request to this repository, make sure that:
- Authorship of the code is properly attributed. You need to agree with the [Developer Certificate of Origin](https://developercertificate.org/) to contribute. Some important points:
  - Make sure your username and user email are [set up](https://docs.github.com/en/get-started/getting-started-with-git/setting-your-username-in-git).
  - If you commit parts of code which are not yours, make sure you are allowed to do so (in terms of license) and respect all the requirements of the license, such as Attribution.
  - If you are not the author of the commit, indicate authorship with `--author` flag of `git commit` command.
  - Try to divide your work into commits done by a single author. If you decide to use a single commit for a multi-author work, use the [Co-authored-by syntax](https://docs.github.com/en/pull-requests/committing-changes-to-your-project/creating-and-editing-commits/creating-a-commit-with-multiple-authors) in the commit message.
- Your commits are signed, indicating compliance with the DCO: use `git commit -s` to ensure that.
- The code is formatted according to the `.clang-format` file in the root of the repository, use `find -iname *.cpp -o -iname *.hpp -o -iname *.c -o -iname *.h | xargs clang-format -style=file -i` command from the `o3de-extras/Gems/ROS2` directory to format the code.

### Additional information

We recommend you to read [O3DE contribution guide](https://github.com/o3de/community/blob/main/CONTRIBUTING.md) and [ROS contribution guide](https://docs.ros.org/en/humble/Contributing.html) as well.
