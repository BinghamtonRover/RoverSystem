# Contributing to Paho

Thanks for your interest in this project!

You can contribute bugfixes and new features by sending pull requests through GitHub.

## Legal

In order for your contribution to be accepted, it must comply with the Eclipse Foundation IP policy.

Please read the [Eclipse Foundation policy on accepting contributions via Git](http://wiki.eclipse.org/Development_Resources/Contributing_via_Git).

1. Sign the [Eclipse CLA](http://www.eclipse.org/legal/CLA.php)
  1. Register for an Eclipse Foundation User ID. You can register [here](https://dev.eclipse.org/site_login/createaccount.php).
  2. Log into the [Projects Portal](https://projects.eclipse.org/), and click on the '[Eclipse CLA](https://projects.eclipse.org/user/sign/cla)' link.
2. Go to your [account settings](https://dev.eclipse.org/site_login/myaccount.php#open_tab_accountsettings) and add your GitHub username to your account.
3. Make sure that you _sign-off_ your Git commits in the following format:
  ``` Signed-off-by: John Smith <johnsmith@nowhere.com> ``` This is usually at the bottom of the commit message. You can automate this by adding the '-s' flag when you make the commits. e.g.   ```git commit -s -m "Adding a cool feature"```
4. Ensure that the email address that you make your commits with is the same one you used to sign up to the Eclipse Foundation website with.

## Contributing a change

1. [Fork the repository on GitHub](https://github.com/eclipse/paho.mqtt.python/fork)
2. Clone the forked repository onto your computer: ``` git clone
   https://github.com/<your username>/paho.mqtt.python.git ```
3. Most changes will go to branch ``master``. This include both bug fixes and
   new features. Bug fixes are committed to ``master`` and if required,
   cherry-picked to the release branch.

   The only changes that goes directly to the release branch (``1.4``,
   ``1.5``, ...) are bug fixe that does not apply to ``master`` (e.g. because
   there are fixed on master by a refactoring, or any other huge change we do
   not want to cherry-pick to the release branch).
4. Create a new branch from the latest ```master``` branch
   with ```git checkout -b YOUR_BRANCH_NAME origin/master```
5. Make your changes
6. Ensure that all new and existing tests pass by running ```tox```
7. Commit the changes into the branch: ``` git commit -s ``` Make sure that
   your commit message is meaningful and describes your changes correctly.
8. If you have a lot of commits for the change, squash them into a single / few
   commits.
9. Push the changes in your branch to your forked repository.
10. Finally, go to
	[https://github.com/eclipse/paho.mqtt.python](https://github.com/eclipse/paho.mqtt.python)
	and create a pull request from your "YOUR_BRANCH_NAME" branch to the
	``master`` (or release branch if applicable) to request review and
	merge of the commits in your pushed branch.


What happens next depends on the content of the patch. If it is 100% authored
by the contributor and is less than 1000 lines (and meets the needs of the
project), then it can be pulled into the main repository. If not, more steps
are required. These are detailed in the
[legal process poster](http://www.eclipse.org/legal/EclipseLegalProcessPoster.pdf).



## Developer resources:


Information regarding source code management, builds, coding standards, and
more.

- [https://projects.eclipse.org/projects/iot.paho/developer](https://projects.eclipse.org/projects/iot.paho/developer)

Contact:
--------

Contact the project developers via the project's development
[mailing list](https://dev.eclipse.org/mailman/listinfo/paho-dev).

Search for bugs:
----------------

This project uses [Github](https://github.com/eclipse/paho.mqtt.python/issues)
to track ongoing development and issues.

Create a new bug:
-----------------

Be sure to search for existing bugs before you create another one. Remember
that contributions are always welcome!

- [Create new Paho bug](https://github.com/eclipse/paho.mqtt.python/issues)
