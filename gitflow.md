## Gitflow

### Get develop at the start of the day
`git checkout develop`

`git pull origin develop`

`git checkout feature_initials`

`git merge develop`

### Throughout the day
add/commit/push whatever to your_repository feature_initials

### End of day or whenever you have something new working
`git checkout develop`

`git merge feature_initials`

`git push origin develop`
