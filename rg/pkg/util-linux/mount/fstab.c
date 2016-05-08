/* 1999-02-22 Arkadiusz Mi�kiewicz <misiek@misiek.eu.org>
 * - added Native Language Support
 * Sun Mar 21 1999 - Arnaldo Carvalho de Melo <acme@conectiva.com.br>
 * - fixed strerr(errno) in gettext calls
 */

#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "mntent.h"
#include "fstab.h"
#include "sundries.h"		/* for xmalloc() etc */
#include "nls.h"

#define streq(s, t)	(strcmp ((s), (t)) == 0)

#define PROC_MOUNTS		"/proc/mounts"


/* Information about mtab. ------------------------------------*/
static int have_mtab_info = 0;
static int var_mtab_does_not_exist = 0;
static int var_mtab_is_a_symlink = 0;

static void
get_mtab_info(void) {
     struct stat mtab_stat;

     if (!have_mtab_info) {
	  if (lstat(MOUNTED, &mtab_stat))
	       var_mtab_does_not_exist = 1;
	  else if (S_ISLNK(mtab_stat.st_mode))
	       var_mtab_is_a_symlink = 1;
	  have_mtab_info = 1;
     }
}

int
mtab_does_not_exist(void) {
     get_mtab_info();
     return var_mtab_does_not_exist;
}

int
mtab_is_a_symlink(void) {
     get_mtab_info();
     return var_mtab_is_a_symlink;
}

int
mtab_is_writable() {
     static int ret = -1;

     /* Should we write to /etc/mtab upon an update?
	Probably not if it is a symlink to /proc/mounts, since that
	would create a file /proc/mounts in case the proc filesystem
	is not mounted. */
     if (mtab_is_a_symlink())
	  return 0;

     if (ret == -1) {
	  int fd = open(MOUNTED, O_RDWR | O_CREAT, 0644);
	  if (fd >= 0) {
	       close(fd);
	       ret = 1;
	  } else
	       ret = 0;
     }
     return ret;
}

/* Contents of mtab and fstab ---------------------------------*/

struct mntentchn mounttable, fstab;
static int got_mtab = 0;
static int got_fstab = 0;

static void read_mounttable(void), read_fstab(void);

struct mntentchn *
mtab_head() {
     if (!got_mtab)
	  read_mounttable();
     return &mounttable;
}

struct mntentchn *
fstab_head() {
     if (!got_fstab)
	  read_fstab();
     return &fstab;
}

static void
read_mntentchn(mntFILE *mfp, const char *fnam, struct mntentchn *mc0) {
	struct mntentchn *mc = mc0;
	struct mntent *mnt;

	while ((mnt = my_getmntent (mfp)) != NULL) {
	    if (!streq (mnt->mnt_type, MNTTYPE_IGNORE)) {
		mc->nxt = (struct mntentchn *) xmalloc(sizeof(*mc));
		mc->nxt->prev = mc;
		mc = mc->nxt;
		mc->mnt_fsname = mnt->mnt_fsname;
		mc->mnt_dir = mnt->mnt_dir;
		mc->mnt_type = mnt->mnt_type;
		mc->mnt_opts = mnt->mnt_opts;
		mc->nxt = NULL;
	    }
	}
	mc0->prev = mc;
	if (ferror (mfp->mntent_fp)) {
		int errsv = errno;
		error(_("warning: error reading %s: %s"), fnam, strerror (errsv));
		mc0->nxt = mc0->prev = NULL;
	}
	my_endmntent(mfp);
}

/*
 * Read /etc/mtab.  If that fails, try /proc/mounts.
 * This produces a linked list. The list head mounttable is a dummy.
 * Return 0 on success.
 */
static void
read_mounttable() {
     mntFILE *mfp;
     const char *fnam;
     struct mntentchn *mc = &mounttable;

     got_mtab = 1;
     mc->nxt = mc->prev = NULL;

     fnam = MOUNTED;
     mfp = my_setmntent (fnam, "r");
     if (mfp == NULL || mfp->mntent_fp == NULL) {
	  int errsv = errno;
	  fnam = PROC_MOUNTS;
	  mfp = my_setmntent (fnam, "r");
	  if (mfp == NULL || mfp->mntent_fp == NULL) {
	       error(_("warning: can't open %s: %s"), MOUNTED, strerror (errsv));
	       return;
	  }
	  if (verbose)
	       printf (_("mount: could not open %s - using %s instead\n"),
		       MOUNTED, PROC_MOUNTS);
     }
     read_mntentchn(mfp, fnam, mc);
}

static void
read_fstab() {
     mntFILE *mfp = NULL;
     const char *fnam;
     struct mntentchn *mc = &fstab;

     got_fstab = 1;
     mc->nxt = mc->prev = NULL;

     fnam = _PATH_FSTAB;
     mfp = my_setmntent (fnam, "r");
     if (mfp == NULL || mfp->mntent_fp == NULL) {
     	  int errsv = errno;
	  error(_("warning: can't open %s: %s"), _PATH_FSTAB, strerror (errsv));
	  return;
     }
     read_mntentchn(mfp, fnam, mc);
}
     

/* Given the name NAME, try to find it in mtab.  */ 
struct mntentchn *
getmntfile (const char *name) {
    struct mntentchn *mc;

    for (mc = mtab_head()->nxt; mc; mc = mc->nxt)
        if (streq (mc->mnt_dir, name) || (streq (mc->mnt_fsname, name)))
	    break;

    return mc;
}

/*
 * Given the name NAME, and the place MCPREV we found it last time,
 * try to find more occurrences.
 */ 
struct mntentchn *
getmntfilesbackward (const char *name, struct mntentchn *mcprev) {
    struct mntentchn *mc, *mh;

    mh = mtab_head();
    if (!mcprev)
	mcprev = mh;
    for (mc = mcprev->prev; mc && mc != mh; mc = mc->prev)
        if (streq (mc->mnt_dir, name) || (streq (mc->mnt_fsname, name)))
	    return mc;

    return NULL;
}

/* Given the name FILE, try to find the option "loop=FILE" in mtab.  */ 
struct mntentchn *
getmntoptfile (const char *file)
{
     struct mntentchn *mc;
     char *opts, *s;
     int l;

     if (!file)
	  return NULL;

     l = strlen(file);

     for (mc = mtab_head()->nxt; mc; mc = mc->nxt)
	  if ((opts = mc->mnt_opts) != NULL
	      && (s = strstr(opts, "loop="))
	      && !strncmp(s+5, file, l)
	      && (s == opts || s[-1] == ',')
	      && (s[l+5] == 0 || s[l+5] == ','))
	       return mc;

     return NULL;
}

/* Find the entry (SPEC,FILE) in fstab */
struct mntentchn *
getfsspecfile (const char *spec, const char *file) {
    struct mntentchn *mc;

    for (mc = fstab_head()->nxt; mc; mc = mc->nxt)
	if (streq (mc->mnt_dir, file) && streq (mc->mnt_fsname, spec))
	     return mc;
    for (mc = fstab_head()->nxt; mc; mc = mc->nxt)
	if ((streq (mc->mnt_dir, file) ||
	     streq (canonicalize(mc->mnt_dir), file))
	    && (streq (mc->mnt_fsname, spec) ||
		streq (canonicalize(mc->mnt_fsname), spec)))
	     break;
    return mc;
}

/* Find the dir FILE in fstab.  */
struct mntentchn *
getfsfile (const char *file) {
    struct mntentchn *mc;

    for (mc = fstab_head()->nxt; mc; mc = mc->nxt)
        if (streq (mc->mnt_dir, file))
	    break;

    return mc;
}

/* Find the device SPEC in fstab.  */
struct mntentchn *
getfsspec (const char *spec)
{
    struct mntentchn *mc;

    for (mc = fstab_head()->nxt; mc; mc = mc->nxt)
        if (streq (mc->mnt_fsname, spec))
	    break;

    return mc;
}

/* Find the uuid UUID in fstab. */
struct mntentchn *
getfsuuidspec (const char *uuid)
{
    struct mntentchn *mc;

    for (mc = fstab_head()->nxt; mc; mc = mc->nxt)
	if (strncmp (mc->mnt_fsname, "UUID=", 5) == 0
	    && streq(mc->mnt_fsname + 5, uuid))
	    break;

    return mc;
}

/* Find the label LABEL in fstab. */
struct mntentchn *
getfsvolspec (const char *label)
{
    struct mntentchn *mc;

    for (mc = fstab_head()->nxt; mc; mc = mc->nxt)
	if (strncmp (mc->mnt_fsname, "LABEL=", 6) == 0
	    && streq(mc->mnt_fsname + 6, label))
	    break;

    return mc;
}

/* Updating mtab ----------------------------------------------*/

/* Flag for already existing lock file. */
static int we_created_lockfile = 0;

/* Flag to indicate that signals have been set up. */
static int signals_have_been_setup = 0;

/* Ensure that the lock is released if we are interrupted.  */
static void
handler (int sig) {
     die (EX_USER, "%s", sys_siglist[sig]);
}

static void
setlkw_timeout (int sig) {
     /* nothing, fcntl will fail anyway */
}

/* Create the lock file.
   The lock file will be removed if we catch a signal or when we exit. */
/* The old code here used flock on a lock file /etc/mtab~ and deleted
   this lock file afterwards. However, as rgooch remarks, that has a
   race: a second mount may be waiting on the lock and proceed as
   soon as the lock file is deleted by the first mount, and immediately
   afterwards a third mount comes, creates a new /etc/mtab~, applies
   flock to that, and also proceeds, so that the second and third mount
   now both are scribbling in /etc/mtab.
   The new code uses a link() instead of a creat(), where we proceed
   only if it was us that created the lock, and hence we always have
   to delete the lock afterwards. Now the use of flock() is in principle
   superfluous, but avoids an arbitrary sleep(). */

/* Where does the link point to? Obvious choices are mtab and mtab~~.
   HJLu points out that the latter leads to races. Right now we use
   mtab~.<pid> instead. */
#define MOUNTLOCK_LINKTARGET	MOUNTED_LOCK "%d"

void
lock_mtab (void) {
	int tries = 3;
	char *linktargetfile;

	if (!signals_have_been_setup) {
		int sig = 0;
		struct sigaction sa;

		sa.sa_handler = handler;
		sa.sa_flags = 0;
		sigfillset (&sa.sa_mask);
  
		while (sigismember (&sa.sa_mask, ++sig) != -1
		       && sig != SIGCHLD) {
			if (sig == SIGALRM)
				sa.sa_handler = setlkw_timeout;
			else
				sa.sa_handler = handler;
			sigaction (sig, &sa, (struct sigaction *) 0);
		}
		signals_have_been_setup = 1;
	}

	/* somewhat clumsy, but some ancient systems do not have snprintf() */
	/* use 20 as upper bound for the length of %d output */
	linktargetfile = xmalloc(strlen(MOUNTLOCK_LINKTARGET) + 20);
	sprintf(linktargetfile, MOUNTLOCK_LINKTARGET, getpid ());

	/* Repeat until it was us who made the link */
	while (!we_created_lockfile) {
		struct flock flock;
		int errsv, fd, i, j;

		i = open (linktargetfile, O_WRONLY|O_CREAT, 0);
		if (i < 0) {
			int errsv = errno;
			/* linktargetfile does not exist (as a file)
			   and we cannot create it. Read-only filesystem?
			   Too many files open in the system? Filesystem full? */
			die (EX_FILEIO, _("can't create lock file %s: %s "
			     "(use -n flag to override)"),
			     linktargetfile, strerror (errsv));
		}
		close(i);

		j = link(linktargetfile, MOUNTED_LOCK);
		errsv = errno;

		(void) unlink(linktargetfile);

		if (j < 0 && errsv != EEXIST) {
			die (EX_FILEIO, _("can't link lock file %s: %s "
			     "(use -n flag to override)"),
			     MOUNTED_LOCK, strerror (errsv));
		}

		fd = open (MOUNTED_LOCK, O_WRONLY);

		if (fd < 0) {
			int errsv = errno;
			/* Strange... Maybe the file was just deleted? */
			if (errno == ENOENT && tries-- > 0)
				continue;
			die (EX_FILEIO, _("can't open lock file %s: %s "
			     "(use -n flag to override)"),
			     MOUNTED_LOCK, strerror (errsv));
		}

		flock.l_type = F_WRLCK;
		flock.l_whence = SEEK_SET;
		flock.l_start = 0;
		flock.l_len = 0;

		if (j == 0) {
			/* We made the link. Now claim the lock. */
			if (fcntl (fd, F_SETLK, &flock) == -1) {
				if (verbose) {
				    int errsv = errno;
				    printf(_("Can't lock lock file %s: %s\n"),
					   MOUNTED_LOCK, strerror (errsv));
				}
				/* proceed anyway */
			}
			we_created_lockfile = 1;
		} else {
			static int tries = 0;

			/* Someone else made the link. Wait. */
			alarm(LOCK_TIMEOUT);
			if (fcntl (fd, F_SETLKW, &flock) == -1) {
				int errsv = errno;
				die (EX_FILEIO, _("can't lock lock file %s: %s"),
				     MOUNTED_LOCK, (errno == EINTR) ?
				     _("timed out") : strerror (errsv));
			}
			alarm(0);
			/* Limit the number of iterations - maybe there
			   still is some old /etc/mtab~ */
			if (tries++ > 3) {
				if (tries > 5)
					die (EX_FILEIO, _("Cannot create link %s\n"
					    "Perhaps there is a stale lock file?\n"),
					     MOUNTED_LOCK);
				sleep(1);
			}
		}

		close(fd);
	}
}

/* Remove lock file.  */
void
unlock_mtab (void) {
	if (we_created_lockfile) {
		unlink (MOUNTED_LOCK);
		we_created_lockfile = 0;
	}
}

/*
 * Update the mtab.
 *  Used by umount with null INSTEAD: remove any DIR entries.
 *  Used by mount upon a remount: update option part,
 *   and complain if a wrong device or type was given.
 *   [Note that often a remount will be a rw remount of /
 *    where there was no entry before, and we'll have to believe
 *    the values given in INSTEAD.]
 */

void
update_mtab (const char *dir, struct mntent *instead) {
     struct mntent *mnt;
     struct mntent *next;
     struct mntent remnt;
     int added = 0;
     mntFILE *mfp, *mftmp;

     if (mtab_does_not_exist() || mtab_is_a_symlink())
	  return;

     lock_mtab();

     mfp = my_setmntent(MOUNTED, "r");
     if (mfp == NULL || mfp->mntent_fp == NULL) {
     	  int errsv = errno;
	  error (_("cannot open %s (%s) - mtab not updated"),
		 MOUNTED, strerror (errsv));
	  goto leave;
     }

     mftmp = my_setmntent (MOUNTED_TEMP, "w");
     if (mftmp == NULL || mfp->mntent_fp == NULL) {
     	  int errsv = errno;
	  error (_("cannot open %s (%s) - mtab not updated"),
		 MOUNTED_TEMP, strerror (errsv));
	  goto leave;
     }
  
     while ((mnt = my_getmntent (mfp))) {
	  if (streq (mnt->mnt_dir, dir)
#if 0
	      /* Matthew Wilcox <willy@odie.barnet.ac.uk> */
	      /* This is meant for Patch 212 on Jitterbug,
		 still in incoming, to allow remounting
		 on a different directory. */
	      || (instead && instead->mnt_fsname &&
		  (!streq (instead->mnt_fsname, "none")) &&
		  (streq (mnt->mnt_fsname, instead->mnt_fsname)))
#endif
		  ) {
	       added++;
	       if (instead) {	/* a remount */
		    remnt = *instead;
		    next = &remnt;
		    remnt.mnt_fsname = mnt->mnt_fsname;
		    remnt.mnt_type = mnt->mnt_type;
		    if (instead->mnt_fsname
			&& !streq(mnt->mnt_fsname, instead->mnt_fsname))
			 printf(_("mount: warning: cannot change "
				"mounted device with a remount\n"));
		    else if (instead->mnt_type
			     && !streq(instead->mnt_type, "unknown")
			     && !streq(mnt->mnt_type, instead->mnt_type))
			 printf(_("mount: warning: cannot change "
				"filesystem type with a remount\n"));
	       } else
		    next = NULL;
	  } else
	       next = mnt;
	  if (next && my_addmntent(mftmp, next) == 1) {
	       int errsv = errno;
	       die (EX_FILEIO, _("error writing %s: %s"),
		    MOUNTED_TEMP, strerror (errsv));
	  }
     }
     if (instead && !added && my_addmntent(mftmp, instead) == 1) {
     	  int errsv = errno;
	  die (EX_FILEIO, _("error writing %s: %s"),
	       MOUNTED_TEMP, strerror (errsv));
    }

     my_endmntent (mfp);
     if (fchmod (fileno (mftmp->mntent_fp), S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH) < 0) {
     	  int errsv = errno;
	  fprintf(stderr, _("error changing mode of %s: %s\n"), MOUNTED_TEMP,
		  strerror (errsv));
     }
     my_endmntent (mftmp);

     if (rename (MOUNTED_TEMP, MOUNTED) < 0) {
     	  int errsv = errno;
	  fprintf(stderr, _("can't rename %s to %s: %s\n"), MOUNTED_TEMP, MOUNTED,
		  strerror(errsv));
     }

leave:
     unlock_mtab();
}
