# Github Pull Request Workflow

This document defines the workflow of pull requests for Citros Garden

## Defenitions

* Origin - the repository on GitHub. If you clone it, it should be already configured with the `git remote` command.

* Local - the repository on your local machine.

## Workflow

1. Checkout to the new branch:

        git checkout -b new_branch_name

2. Fix the issue / Change the code, and commit when needed:

		git add -A
		git commit -m ”description of the changes”

3. When finished, add and commit again

4. Push the commits to the origin:

        git push origin new_branch_name

5. On GitHub, switch to the new branch

6. On Github, open Pull Request - Contribute -> open pull request

7. Assign someone to review the code

8. Add labels and assign yourself

9. Wait for code review confirmation

10. When the code has been reviewed and everything is OK, merge the PR with `Squash and Merge` Option.

11. In your local repository, check out the main branch:

        git checkout main

12. Then pull the new main branch from the origin:

	    git pull origin main

13. Delete the PR branch on your local PC:

	    git branch -D new_branch_name
