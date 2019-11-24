## Gitflow
We will all be working on our own feature branches. These branches will
be pushed to our own public repositories in github. We will be merging the 
changes from our feature branches into develop branches everday. We will
be pushing our changes to develop everyday after we finish working. No one
pushes directly into master. One of us will be merging changes from develop
into master periodically when we hit milestones. Push as frequently as possible
to develop to avoid long unhandled merge conflicts.

### Get develop at the start of the day
`git checkout develop`

`git pull origin develop`

`git checkout feature_initials`

`git merge develop`

### Throughout the day
add/commit/push whatever to your_repository feature_initials

### End of day or whenever you have something new working
`git checkout develop`

`git pull origin develop`

`git merge feature_initials`

`git push origin develop`
