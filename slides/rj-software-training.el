;;; rj-software-training.el --- generation scripts for software-training -*- lexical-binding: t -*-

;; Copyright (C) 2016-2017 Jay Kamat
;; Author: Jay Kamat <jaygkamat@gmail.com>
;; Version: 2.0
;; Keywords: robojackets,training,revealjs,gfm
;; URL: https://github.com/RoboJackets/software-training
;; Package-Requires: ((emacs "25.0") (f) (ox-gfm) (htmlize) (ox-reveal))
;;; Commentary:
;; Creates and builds org project files for robojackets software training.

;; To run: cask eval "(progn (require 'rj-software-training) (rj-software-training-publish))" in the root of the project.

;;; Dependencies
;; Org publishing
(require 'ox-publish)
;; Org -> Slides
(require 'ox-reveal)
;; Org -> gh md
(require 'ox-gfm)
;; file manipulation lib
(require 'f)
;; Hash table (and more) library
(require 'subr-x)

;;; Code:
;;;; Project root def
(defconst +RJ-PROJ-ROOT+ (file-name-directory load-file-name))

;;;; Project Definition
;; Force htmlize to activate even in nogui mode:
;; https://stackoverflow.com/questions/3591337/emacs-htmlize-in-batch-mode
;; sebastien.kirche.free.fr/emacs_stuff/elisp/my-htmlize.el
;; Get fancy colors! (but this will screw up your native emacs install)
(when noninteractive
  ;; Don't run in interactive mode to avoid breaking your colors
  (custom-set-faces
    ;; Draculized minimal: https://github.com/jgkamat/darculized
    ;; TODO find out why default face is not applying.
    '(default                      ((t (:foreground "#909396" :background "#262626"))))
    '(font-lock-builtin-face       ((t (:foreground "#598249"))))
    '(font-lock-comment-face       ((t (:foreground "#5e6263"))))
    '(font-lock-constant-face      ((t (:foreground "#15968D"))))
    '(font-lock-function-name-face ((t (:foreground "#2F7BDE"))))
    '(font-lock-keyword-face       ((t (:foreground "#598249"))))
    '(font-lock-string-face        ((t (:foreground "#15968D"))))
    '(font-lock-type-face		       ((t (:foreground "#598249"))))
    '(font-lock-variable-name-face ((t (:foreground "#2F7BDE"))))
    '(font-lock-warning-face       ((t (:foreground "#bd3832" :weight bold)))))
  (setq htmlize-use-rgb-map 'force)
  (require 'htmlize))

(let ((proj-base +RJ-PROJ-ROOT+))
  (setq org-publish-project-alist
    `(("rj-slides"
        :recursive t
        :with-toc nil
        :with-timestamps nil
        :time-stamp-file nil
        :with-tags nil
        :with-author nil
        :with-date nil
        :with-todo-keywords nil
        :section-numbers nil
        :reveal-history nil
        :reveal-control nil
        :reveal-hlevel "0"
        :reveal-plugins "(notes pdf)"
        :reveal-speed "fast"
        :reveal-trans "linear"
        :reveal-theme "white"
        :reveal-width 1440
        :reveal-height 800
        :base-directory ,proj-base
        :publishing-directory ,(f-join proj-base ".." "html" "slides")
        :publishing-function org-reveal-publish-to-reveal
        :exclude-tags ("docs"))
       ("rj-docs"
         :recursive t
         :with-toc nil
         :with-tags nil
         :with-author nil
         :with-date nil
         :with-todo-keywords nil
         :section-numbers nil
         :with-timestamps nil
         :time-stamp-file nil
         :base-directory ,proj-base
         :publishing-directory ,(f-join proj-base ".." "html" "docs")
         :publishing-function org-gfm-publish-to-gfm
         :exclude-tags ("slides"))
       ("rj-overview"
         :recursive t
         :with-todo-keywords nil
         :with-timestamps nil
         :time-stamp-file nil
         :base-directory ,proj-base
         :html-head-extra "<link rel=\"stylesheet\" type=\"text/css\" href=\"https://jgkamat.github.io/src/jgkamat.css\">"
         :publishing-directory ,(f-join proj-base ".." "html" "web")
         :publishing-function org-html-publish-to-html))
    org-reveal-root "https://robojackets.github.io/reveal.js/"
    org-reveal-margin "0.15"))


;;;; Org Babel Settings
(require 'ob-python)
(require 'ob-C)
(setq org-babel-python-command "python3")
;; Make indentation actually matter in org src blocks
(setq org-src-preserve-indentation nil)

(when noninteractive
  ;; Don't ask for evaluation
  ;; (WARNING THIS WILL COMPILE/RUN CODE ON YOUR COMPUTER)
  ;; DO NOT RUN NONINTERACTIVELY IF YOU DO NOT ACCEPT THIS
  (defun my-org-confirm-babel-evaluate (lang body)
    "Stop org mode from complaining about python.
LANG language input
BODY code body"
    (not (member lang '("emacs-lisp" "python" "dot" "sh" "C" "C++"))))
  (setq org-confirm-babel-evaluate #'my-org-confirm-babel-evaluate))

;;;; Helper scripts for automatic high level TOC

(defun rj-org-file-to-link (filename)
  "Turn an org FILENAME into a org link."
  (with-temp-buffer
    (insert-file-contents filename)
    (concat
      "[[" filename "]"
      "[" (or
            (first (plist-get (org-export-get-environment) :title))
            "unknown")
      "]]")))

(defun rj-generate-project-files (relative)
  "Generate rj org files.
RELATIVE path that files should be relative to"
  (mapcar
    ;; Make paths relative
    (lambda (file) (f-relative file relative))
    (f-files
      ;; Get all non-hidden org files from root (that aren't an overview)
      +RJ-PROJ-ROOT+
      (lambda (file)
        (and
          (not (f-child-of? file (f-join +RJ-PROJ-ROOT+ "overview")))
          (not (f-hidden? file))
          (equal (f-ext file) "org")))
      t)))

(defun rj-generate-project-toc ()
  "Generate file level TOC via org links."
  ;; TODO don't hardcode "overview" as the relative folder here.
  (let ((project-files (rj-generate-project-files (f-join +RJ-PROJ-ROOT+ "overview")))
         (collection-hasht (make-hash-table :test #'equal))
         (result ""))
    ;; Add files to a hashtable, to group them by folder
    (dolist (file project-files)
      (let ((key (f-filename (f-dirname file))))
        (setf
          (gethash key collection-hasht)
          (append `(,file) (gethash key collection-hasht)))))
    (dolist (key (sort (hash-table-keys collection-hasht) #'string<))
      ;; Generate headers
      (setq result (concat result
                     "* " key "\n"))
      (dolist (file (gethash key collection-hasht))
        ;; Generate entries
        (setq result (concat result "1. "
                       (rj-org-file-to-link file)
                       "\n"))))
    result))

;;;; Publishing script

(defun rj-software-training-publish ()
  "Simple script to export this project."
  (interactive)
  ;; Don't make backup files when generating (cask)
  (let ((make-backup-files nil))
    (org-publish-all t)))

(provide 'rj-software-training)

;;; rj-software-training.el ends here
