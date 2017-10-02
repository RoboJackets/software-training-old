;;; rj-software-training.el --- generation scripts for software-training

;; Copyright (C) 2016-2017 Jay Kamat
;; Author: Jay Kamat <jaygkamat@gmail.com>
;; Version: 1.0
;; Keywords: robojackets,training,revealjs,gfm
;; URL: https://github.com/RoboJackets/software-training
;; Package-Requires: ((emacs "25.0") (ox-gfm) (htmlize) (ox-reveal))
;;; Commentary:
;;; Creates and builds org project files for robojackets software training.

;; To run: cask eval "(progn (require 'rj-software-training) (rj-software-training-publish))" in the root of the project.

(require 'ox-reveal)
(require 'ox-publish)
(require 'ox-gfm)

;;; Code:

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

(let ((proj-base (file-name-directory load-file-name)))
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
        :base-directory ,(concat proj-base ".")
        :publishing-directory ,(concat proj-base "../html/slides/")
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
         :base-directory ,(concat proj-base ".")
         :publishing-directory ,(concat proj-base "../html/docs/")
         :publishing-function org-gfm-publish-to-gfm
         :exclude-tags ("slides")))
    org-reveal-root "https://robojackets.github.io/reveal.js/"
    org-reveal-margin "0.15"))


(require 'ob-python)
(require 'ob-C)
(require 'ob-shell)
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

;; Add additional font lock keywords to sh-mode
(add-hook 'sh-mode-hook
  (lambda ()
    (font-lock-add-keywords nil
      '(("^\\s-*\\(git\\|ls\\|cat\\)\\>" . 'font-lock-keyword-face)))))

(defun rj-software-training-publish ()
  "Simple script to export this project."
  (interactive)
  ;; Don't make backup files when generating (cask)
  (let ((make-backup-files nil))
    (org-publish-all t)))

(provide 'rj-software-training)

;;; rj-software-training.el ends here
