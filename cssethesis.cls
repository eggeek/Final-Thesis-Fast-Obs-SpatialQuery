%%
%% This is file `cssethesis.cls',
%% generated with the docstrip utility.
%%
%% The original source files were:
%%
%% cssethesis.dtx  (with options: `class')
%% 
%% IMPORTANT NOTICE:
%% 
%% For the copyright see the source file.
%% 
%% Any modified versions of this file must be renamed
%% with new filenames distinct from cssethesis.cls.
%% 
%% For distribution of the original source see the terms
%% for copying and modification in the file cssethesis.dtx.
%% 
%% This generated file may be distributed as long as the
%% original source files, as listed above, are part of the
%% same distribution. (The sources need not necessarily be
%% in the same archive or directory.)
%%
%%  This is cssethesis.dtx $Revision: 1.7 $.
%%
%%  Please note:  This document class is not compatable with the
%%                old LaTeX 2.09.  It uses LaTeX2e specific commands.
%%
%%  Please send bug reports to:
%%
%%  David.Squire@csse.monash.edu.au (DMS)
%%
%% Revision history:
%% 20051018 DMS Changes in version 1.7:
%% -    fixed up headers and footers in draft mode, so that draft mode puts
%%      a draft notice and timestamp in the footer, and leaves the standard
%%      headers alone.
%% -    fixed size of "DRAFT" text so that it fits in the margin
%% -    removed command \thesisdepartmentname, and provided a default value
%%      for \@thesisdepartment, that can be reset using the
%%      \thesisdepartment command. This is to cope with the new "Clayton
%% -    removed command \thesisauthoraddress
%%      School of Information Technology" and "Caulfield School of
%%      Information Technology"
%% -    Changed BSE coursecode to 2770
%% - Changed references to "School of Computer Science and Software
%%      Engineering" in documentation
%% 20040207 DMS New in version 1.6:
%%  -   Removed use of ulem from thesissignaturepage (which is not used in
%%      the example document).
%% 20040206
%%  -   Fixed "Underfull \hbox error in the declaration.
%%  -   Removed loading of ulem package, which was not being used.
%%  -   Added support for the new Master of Computer Science (minor thesis).
%%  -   Fixed line spacing of thesis title on abstract and declaration
%%      pages. This was a problem for multi-line titles.
%%  -   Added pdflatex option, so that a pdf crest is used, and the
%%      appropriate option passed to the graphics package. The user must
%%      then compile with pdflatex, and use pdf figures.
%% 20040205 DMS
%%  -   Now catch any openany or openright options. The frontmatter is
%%      forced to openany, and the user's choice used for the mainmatter.
%%      The default is openright.
%%  -   Use geometry package to set page layout.
%%  -   Removed redefinition of chapter.
%%  -   Removed definition of a new page style. Just use defaults available
%%      in book.cls - the user can define new ones with the fancyheadings
%%      pacakge if wanted.
%%  -   Changed abstract page so that it prints the actual qualifications
%%      of the author, not the one which the thesis will get them if passed!
%%  -   stopped forcing of |oneside| option to book.cls - now it should get
%%      whatever the user specifies in the documentclass options. Note that
%%      |twoside| is the default for book.cls.
%% 20040204 DMS
%%   -  Fixed bugs introduced by attempt to support literature review,
%%      which suppressed printing of email addresses, associate supervisor
%%      details, etc.
%%   -  Changed name of PhD thesis from "Dissertation" to "Thesis".
%%   -  Removed supervisors' email addresses from title page.
%%   -  Added email addresses to the abstract page.
%% 20020727 DMS New in version 1.5:
%%   -  The thebibliography environment was updated to reflect a
%%      more recent book.cls. This fixed a bug in version 1.4 when used with
%%      more recent \LaTeX2e distributions
%%   -  commands defined so that user can set most commonly needed options
%%      without needing to use \renewcommand
%%   -  thesis type options modified, and some added, to reflect the current
%%      courses available in the school
%%   -  command \thesiscoursecode added so that the numerical coursecode can
%%      be output on title page
%%   -  Option nocoursecode added so that the output of the numerical
%%      coursecode can be supressed if desired
%%   -  Title page modified to include Monash Crest
%%   -  Default values set up for month and year, so that setting them is
%%      now optional
%%   -  Option litreview added so that user can indicate that this is a
%%      literature review, rather than the thesis itself. A different title
%%      page in produced if so.
%%   -  commands added to support author's student id and email address
%%   -  commands added to support associate supervisor
%%   -  commands added to support supervisors' email addresses
%%   -  specification of previous degrees made optional

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{cssethesis}[2005/10/18 v1.7 Monash Caulfield and Clayton Schools of Information Technology thesis class]
\newif\if@pdflatex \@pdflatexfalse % Default is false
\DeclareOption{pdflatex}{\@pdflatextrue}
\newif\if@wasopenright \@wasopenrighttrue % true is the default
\DeclareOption{openright}{\@wasopenrighttrue}
\DeclareOption{openany}{\@wasopenrightfalse}
\DeclareOption{thesisdraft}{%
  \AtEndOfClass{% So it will override the definition given below
\let\ps@plain\@empty
    \def\ps@plain{%
     \let\@oddfoot\@empty\let\@evenfoot\@empty
     \def\@oddfoot{\hfil\textsl{Draft of \timestamp}}%
     \def\@evenfoot{\textsl{Draft of \timestamp}\hfil}%
}
\let\ps@headings\@empty
    \def\ps@headings{%
     \let\@oddfoot\@empty\let\@evenfoot\@empty
     \def\@oddfoot{\hfil\textsl{Draft of \timestamp}}%
     \def\@evenfoot{\textsl{Draft of \timestamp}\hfil}%
}
   }
}
\DeclareOption{thesispsdraft}{%
  \AtEndOfClass{% So it will override the definition given below
\let\ps@plain\@empty
    \def\ps@plain{%
\special{!userdict begin /bop-hook{gsave 70 270 translate
          90 rotate /Times-Roman findfont 80 scalefont setfont
          0 0 moveto 0.9 setgray (DRAFT) show grestore}def end}
     \let\@oddfoot\@empty\let\@evenfoot\@empty
     \def\@oddfoot{\hfil\textsl{Draft of \timestamp}}%
     \def\@evenfoot{\textsl{Draft of \timestamp}\hfil}%
}
\let\ps@headings\@empty
    \def\ps@headings{%
     \let\@oddfoot\@empty\let\@evenfoot\@empty
     \def\@oddfoot{\hfil\textsl{Draft of \timestamp}}%
     \def\@evenfoot{\textsl{Draft of \timestamp}\hfil}%
}
}
  }
\DeclareOption{phdthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{for fulfillment}
\newcommand{\thesistype}{Thesis}
\newcommand{\thesisdegree}{Doctor of Philosophy}
\newcommand{\thesisdegreeabbreviation}{PhD}
\newcommand{\thesiscoursecode}{0190}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{mitthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{for fulfillment}
\newcommand{\thesistype}{Thesis}
\newcommand{\thesisdegree}{Master of Information Technology (Research)}
\newcommand{\thesisdegreeabbreviation}{MIT}
\newcommand{\thesiscoursecode}{1895}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{mitminorthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{in partial fulfillment}
\newcommand{\thesistype}{Minor Thesis}
\newcommand{\thesisdegree}{Master of Information Technology (Minor Thesis)}
\newcommand{\thesisdegreeabbreviation}{MIT}
\newcommand{\thesiscoursecode}{3316}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{mitcourseworkthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{in partial fulfillment}
\newcommand{\thesistype}{Minor Thesis}
\newcommand{\thesisdegree}{Master of Information Technology (Coursework)}
\newcommand{\thesisdegreeabbreviation}{MIT}
\newcommand{\thesiscoursecode}{2402}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{mcsminorthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{in partial fulfillment}
\newcommand{\thesistype}{Minor Thesis}
\newcommand{\thesisdegree}{Master of Computer Science (Minor Thesis)}
\newcommand{\thesisdegreeabbreviation}{MCS}
\newcommand{\thesiscoursecode}{3328}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{mdcthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{for fulfillment}
\newcommand{\thesistype}{Thesis}
\newcommand{\thesisdegree}{Master of Digital Communications}
\newcommand{\thesisdegreeabbreviation}{MDC}
\newcommand{\thesiscoursecode}{2406}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{bcomphonoursthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{in partial fulfillment}
\newcommand{\thesistype}{Thesis}
\newcommand{\thesisdegree}{Bachelor of Computing with Honours}
\newcommand{\thesisdegreeabbreviation}{BComp(Hons)}
\newcommand{\thesiscoursecode}{0194}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{bcshonoursthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{in partial fulfillment}
\newcommand{\thesistype}{Thesis}
\newcommand{\thesisdegree}{Bachelor of Computer Science with Honours}
\newcommand{\thesisdegreeabbreviation}{BCompSc(Hons)}
\newcommand{\thesiscoursecode}{1608}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{bdshonoursthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{in partial fulfillment}
\newcommand{\thesistype}{Thesis}
\newcommand{\thesisdegree}{Bachelor of Digital Systems with Honours}
\newcommand{\thesisdegreeabbreviation}{BDigSys(Hons)}
\newcommand{\thesiscoursecode}{1200}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{bshonoursthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{in partial fulfillment}
\newcommand{\thesistype}{Thesis}
\newcommand{\thesisdegree}{Bachelor of Science (Computer Science) with
Honours}
\newcommand{\thesisdegreeabbreviation}{BSc(Hons)}
\newcommand{\thesiscoursecode}{0051}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{bsehonoursthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{in partial fulfillment}
\newcommand{\thesistype}{Thesis}
\newcommand{\thesisdegree}{Bachelor of Software Engineering with
Honours}
\newcommand{\thesisdegreeabbreviation}{BSE(Hons)}
\newcommand{\thesiscoursecode}{2770}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{mcompthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{for fulfillment}
\newcommand{\thesistype}{Thesis}
\newcommand{\thesisdegree}{Master of Computing}
\newcommand{\thesisdegreeabbreviation}{MComp}
\newcommand{\thesiscoursecode}{}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{mcompminorthesis}{%
\ifx\undefined\thesistype
\newcommand{\thesisfulfillment}{in partial fulfillment}
\newcommand{\thesistype}{Thesis}
\newcommand{\thesisdegree}{Master of Computing (Minor)}
\newcommand{\thesisdegreeabbreviation}{MComp}
\newcommand{\thesiscoursecode}{}
\else
\ClassError{cssethesis}{only one thesis type option may be
specified}{}
\fi
}
\DeclareOption{nocoursecode}{%
    \let\thesiscoursecode\@undefined
}
\newif\if@litreview \@litreviewfalse
\DeclareOption{litreview}{%
\@litreviewtrue
\renewcommand{\thesistype}{Literature Review}
}
\DeclareOption{singlespace}{\AtBeginDocument{\singlespacing}}
\DeclareOption{oneandhalfspace}{\AtBeginDocument{\onehalfspacing}}
\DeclareOption{doublespace}{\AtBeginDocument{\doublespacing}}
\DeclareOption{indentfirst}{\let\@afterindentfalse\@afterindenttrue
                            \@afterindenttrue}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{book}}
\ExecuteOptions{oneandhalfspace}
\ProcessOptions
\LoadClass{book}[1994/12/01]
\ifx\undefined\thesistype
  \ClassError{cssethesis}
   {thesis type not specified. Please give a class option to specify the
   thesis type}%
   {Use, for example, \protect\documentclass[phdthesis]{cssethesis}.
   Possibilities are: phdthesis, mitthesis, mitminorthesis,
   mitcourseworkthesis, mcsminorthesis, mdcthesis, bcomphonoursthesis,
   bcshonoursthesis, bdshonoursthesis, bshonoursthesis,bsehonoursthesis}
\fi
\RequirePackage[paper=a4paper,left=30mm,width=150mm,top=25mm,bottom=25mm]{geometry}
\if@pdflatex
\RequirePackage[pdftex]{graphicx}
\else
\RequirePackage[dvips]{graphicx}
\fi
\RequirePackage{setspace}
\newcommand*{\thesisauthor}[1]{\gdef\@thesisauthor{#1}}
\newcommand*{\thesisauthorlastname}[1]{\gdef\@thesisauthorlastname{#1}}
\newcommand*{\thesisauthorpreviousdegrees}[1]{\gdef\@thesisauthorpreviousdegrees{#1}}
\newcommand*{\thesisauthorstudentid}[1]{\gdef\@thesisauthorstudentid{#1}}
\newcommand*{\thesisauthoremail}[1]{\gdef\@thesisauthoremail{#1}}
\newcommand*{\thesismonth}[1]{\gdef\@thesismonth{#1}}
\def\@thesismonth{\ifcase%
\month\or January%
\or February%
\or March%
\or April%
\or May%
\or June%
\or July%
\or August%
\or September%
\or October%
\or November%
\or December\fi} % use current month as default
\newcommand*{\thesisyear}[1]{\gdef\@thesisyear{#1}}
\def\@thesisyear{\number\year} % use current year as default
\newcommand*{\thesistitle}[1]{\gdef\@thesistitle{#1}}
\newcommand*{\thesissupervisor}[1]{\gdef\@thesissupervisor{#1}}
\newcommand*{\thesissupervisoremail}[1]{\gdef\@thesissupervisoremail{#1}}
\newcommand*{\thesisassocsupervisor}[1]{\gdef\@thesisassocsupervisor{#1}}
\newcommand*{\thesisassocsupervisoremail}[1]{\gdef\@thesisassocsupervisoremail{#1}}
%% Set up these ones with default values
\gdef\@thesisdepartment{Clayton School of Information Technology}
\newcommand*{\thesisdepartment}[1]{\gdef\@thesisdepartment{#1}}
\newcommand*{\thesisuniversity}[1]{\gdef\@thesisuniversity{#1}}
\def\@thesisuniversity{Monash University}
\newcommand*{\thesistypist}[1]{\gdef\@thesistypist{#1}}
\def\@thesistypist{the author}
\newcommand*{\thesisdedication}[1]{\gdef\@thesisdedication{#1}}
\newcommand{\thesisdefaultbaselinestretch}{1.5}
\newcommand{\thesistypetypesize}{\large}
\newcommand{\thesistitletypesize}{\Large}
\newcommand{\thesistitlebold}{\textbf{\thesistitletypesize\@thesistitle}}
\newcommand{\thesisstudenttypesize}{\large}
\newcommand{\thesisstudentbold}{\textbf{\thesisstudenttypesize \thesisstudent}}
\newcommand{\thesisdegreetypesize}{\large}
\newcommand{\thesistitlepagefont}{\fontsize{14}{14}\selectfont}
\newcommand{\thesiscopyrightholder}{\@thesisauthor}
\newcommand{\thesiscommittee}{Supervising Committee:}
\newcommand{\thesiscommitteesize}{5}
\newcommand{\thesisvitahead}{Vita}
\newcommand{\thesisvitaheadtypesize}{\huge\bfseries}
\newcommand{\thesisacknowledgmenthead}{Acknowledgments}
\newcommand{\thesisacknowledgmentsigname}{\@thesisauthor}
\newcommand{\thesisacknowledgmentsignametype}{\large}
\newcommand{\thesisacknowledgmentsigplace}%
    {{\slshape
        \@thesisuniversity \\
        \@thesismonth\ \@thesisyear}}
\newcommand{\thesisstudent}{%
\ifx\undefined\@thesisauthorpreviousdegrees
\@thesisauthor
\else
\@thesisauthor, \@thesisauthorpreviousdegrees
\fi
}
\widowpenalty =10000
\clubpenalty  =10000
\brokenpenalty=10000
\renewcommand{\frontmatter}{%
  \cleardoublepage
  \@mainmatterfalse
  \pagenumbering{roman}
  \@openrightfalse
  \pagestyle{plain}}
\renewcommand{\mainmatter}{%
  \cleardoublepage
  \@mainmattertrue
  \pagenumbering{arabic}
  \if@wasopenright
   \@openrighttrue % restore previous setting
  \else
   \@openrightfalse
  \fi
  \pagestyle{headings}}
%% \section{Frontmatter}
\newcommand{\thesisdedicationpage}{%
    \strut \vfill
    \begin{spacing}{1.5}
    \begin{center}
      \@thesisdedication
    \end{center}
    \end{spacing}
    \vfill \strut
    \newpage}
%% \begin{environment}{thesisacknowledgments}
\newenvironment{thesisacknowledgments}
    {%
     \chapter*{\thesisacknowledgmenthead
         \@mkboth{\thesisacknowledgmenthead}
                 {\thesisacknowledgmenthead}}
     \addcontentsline{toc}{chapter}{\thesisacknowledgmenthead}
     \begin{spacing}{1.1}}
    {\nopagebreak\end{spacing} \par\nopagebreak
     \raggedleft
      {\thesisacknowledgmentsignametype\thesisacknowledgmentsigname}
     \\*[10mm]
     \raggedright
      \thesisacknowledgmentsigplace
     \newpage}
\newcommand{\thesistitlepage}{%
\thispagestyle{empty}
\begin{spacing}{1.5}
\begin{center}
\thesistitlebold \\[4mm]
by \\[4mm]
\thesisstudentbold \\
\ifx\undefined\@thesisauthoremail
\relax
\else
\if@litreview
\vspace{-8pt}
\texttt{\@thesisauthoremail}\\
\fi
\fi
\vspace{8mm}
\begin{center}
\includegraphics[width=5cm]{MonashCrest}
\end{center}
\textbf{\thesistypetypesize \thesistype} \\
\if@litreview
\relax
\else
Submitted by \@thesisauthor \\
\thesisfulfillment\ of the Requirements %\\
for the Degree of \\
\fi
\ifx\undefined\thesiscoursecode
\textbf{\thesisdegreetypesize \thesisdegree} \\
\else
\textbf{\thesisdegreetypesize \thesisdegree\ (\thesiscoursecode)} \\
\fi
\vspace{12mm}
\ifx\undefined\@thesissupervisor
\ClassError{cssethesis}{a thesissupervisor must be defined when
the litreview option is used}{}
\else
Supervisor: \@thesissupervisor\\
\fi
\ifx\undefined\@thesisassocsupervisor
\relax
\else
Associate Supervisor: \@thesisassocsupervisor\\
\fi
\strut \vfill
\textbf{\Large \@thesisdepartment} \\
\textbf{\Large \@thesisuniversity} \\%[4mm]
\@thesismonth, \@thesisyear
\end{center}
\end{spacing}
    \newpage}
\newcommand{\thesiscopyrightpage}{%
    \thispagestyle{empty}
    \strut \vfill
    \begin{center}
      \begin{spacing}{2}
      \copyright\ Copyright \\ by \\ \thesiscopyrightholder \\ \@thesisyear
      \end{spacing}
    \end{center}
    \vfill \strut
    \newpage}
\newenvironment{thesisabstract}{%
     \newpage
     \addcontentsline{toc}{chapter}{Abstract}
     \setlength{\unitlength}{1mm}
        \begin{center}
         \begin{spacing}{1.5}
         \thesistitlebold \\[7mm]
         \end{spacing}
         \begin{spacing}{1}
         \@thesisauthor
 \ifx\undefined\@thesisauthorpreviousdegrees
 \\
 \else
  , \@thesisauthorpreviousdegrees \\
 \fi
\ifx\undefined\@thesisauthoremail
\relax
\else
\texttt{\@thesisauthoremail}\\
\fi
         \@thesisuniversity, \@thesisyear \\[7mm]
         Supervisor: \@thesissupervisor \\
 \ifx\undefined\@thesissupervisoremail
\relax
\else
\texttt{\@thesissupervisoremail}\\
\fi
\ifx\undefined\@thesisassocsupervisor
\relax
\else
Associate Supervisor: \@thesisassocsupervisor\\
 \ifx\undefined\@thesisassocsupervisoremail
\relax
\else
\texttt{\@thesisassocsupervisoremail}\\
\fi
\fi
 \vspace*{7mm}
         \textbf{Abstract} \\
         \end{spacing}
        \end{center}
     \vspace{5mm}
     \noindent}
     \newpage
\newcommand{\thesisdeclarationpage}{%
    \begin{center}
      \begin{spacing}{1.5}
      \thesistitlebold
      \end{spacing}
    \end{center}
    \vspace{20mm}
    \begin{center}
    {\bf Declaration}
    \end{center}
     \begin{spacing}{1}
    I declare that this thesis is my own work and has not been
    submitted in any form for another degree or diploma at any university
    or other institute of tertiary education.  Information derived from
    the published and unpublished work of others has been acknowledged in
    the text and a list of references is given.
    \end{spacing}
    \vspace{15mm}
    \hfill \begin{minipage}{67mm}
                     \begin{spacing}{1}
                     \vspace{10mm}
                     \setlength{\unitlength}{1mm}
                     \begin{picture}(67,1)(0,0)
                        \line(1,0){67}
                     \end{picture} \\
                     \@thesisauthor \\
                     \today
                     \end{spacing}
                   \end{minipage}
    \newpage}
\newcommand{\thesissignaturepage}{%
    \begin{center}
      \begin{spacing}{1.5}
      \thesistitlebold
      \end{spacing}
    \end{center}
    \vspace{20mm}
    \begin{center}
      {\bf \thesistype\ Acceptance}
    \end{center}
    \begin{spacing}{1}
    This student's \thesistype, entitled
    \textbf{\@thesistitle}
    has been examined by the undersigned committee of
    examiners and has received full approval for acceptance \thesisfulfillment\
    of the requirements for the degree of \thesisdegree.
    \end{spacing}

    \vspace{15mm}
    \hbox to \hsize{\hfill APPROVAL: \begin{minipage}[b]{67mm}
                     \setlength{\unitlength}{1mm}
                     \begin{picture}(67,-70)
                       \multiput(0,-2)(0,-10)
                         {\thesiscommitteesize}{\line(1,0){67}}
                     \end{picture}
                   \end{minipage} Chief Examiner}
    \newpage}
\newcommand{\thesisshorttitlepage}{%
    \vbox to 4.1in{\vfil
      \hbox to \hsize {Short Title: \thesisshorttitle \hfill
         \@thesisauthorlastname, \thesisdegreeabbreviation\ \@thesisyear
         \hspace{0.5in}}}
    \newpage}
\renewcommand{\listoffigures}{%
  \chapter*{\listfigurename}%
  \addcontentsline{toc}{chapter}{\listfigurename}%
  \@starttoc{lof}
}
\renewcommand{\listoftables}{%
  \chapter*{\listtablename}%
  \addcontentsline{toc}{chapter}{\listtablename}%
  \@starttoc{lot}
}
%% \section{Mainmatter}
\newif\if@appendix\@appendixfalse
\renewcommand{\appendix}{\par
  \setcounter{chapter}{0}%
  \setcounter{section}{0}%
  \renewcommand{\@chapapp}{\appendixname}%
  \renewcommand{\thechapter}{\Alph{chapter}}
  \@appendixtrue}
\def\@chapter[#1]#2{\ifnum \c@secnumdepth >\m@ne
                       \if@mainmatter
                         \refstepcounter{chapter}%
                         \typeout{\@chapapp\space\thechapter.}%
                         \if@appendix
                           \addcontentsline{toc}{chapter}%
                                   {\appendixname\ \thechapter\ #1}%
                         \else
                           \addcontentsline{toc}{chapter}%
                                   {\protect\numberline{\thechapter}#1}%
                         \fi
                       \else
                         \addcontentsline{toc}{chapter}{#1}%
                       \fi
                    \else
                      \addcontentsline{toc}{chapter}{#1}%
                    \fi
                    \chaptermark{#1}%
                    \addtocontents{lof}{\protect\addvspace{10\p@}}%
                    \addtocontents{lot}{\protect\addvspace{10\p@}}%
                    \if@twocolumn
                      \@topnewpage[\@makechapterhead{#2}]%
                    \else
                      \@makechapterhead{#2}%
                      \@afterheading
                    \fi}
\renewcommand{\l@chapter}[2]{%
  \ifnum \c@tocdepth >\m@ne
    \addpenalty{-\@highpenalty}%
    \vskip 1.0em \@plus\p@
    \setlength\@tempdima{1.5em}%
    \begingroup
      \parindent \z@ \rightskip \@pnumwidth
      \parfillskip -\@pnumwidth
      \leavevmode \bfseries
      \advance\leftskip\@tempdima
      \hskip -\leftskip
      #1\nobreak%
      \leaders\hbox{$\m@th \mkern \@dotsep mu.\mkern \@dotsep mu$}\hfill
      \nobreak\hbox to\@pnumwidth{\hss #2}\par
      \penalty\@highpenalty
    \endgroup
  \fi}
\newenvironment{thesisauthorvita}{%
     \clearpage
     \chapter*{\thesisvitahead
         \@mkboth{\uppercase{\thesisvitahead}}
                 {\uppercase{\thesisvitahead}}}
    \addcontentsline{toc}{chapter}{\thesisvitahead}
    \begin{spacing}{\thesisdefaultbaselinestretch}}
    {\end{spacing} \par \vspace{20mm}
     \noindent Permanent Address:
       \begin{minipage}[t]{90mm}
{\@thesisdepartment\\Monash University\\Australia}
       \end{minipage}
     \par \vspace{15mm}
     \noindent This \lowercase\expandafter{\thesistype} was
       typeset with \LaTeXe\footnote{\LaTeXe{} is an extension of \LaTeX.
       \LaTeX{} is a collection of macros for \TeX.  \TeX{} is a
       trademark of the American Mathematical Society.  The macros used in
       formatting this \lowercase\expandafter{\thesistype} were written
       by Glenn Maughan and modified by Dean Thompson and David Squire of Monash University.
    } by \@thesistypist.}
\renewcommand{\bibname}{References}
\renewenvironment{thebibliography}[1]
     {\chapter*{\bibname%
      \addcontentsline{toc}{chapter}{\bibname}
        \@mkboth{\MakeUppercase\bibname}{\MakeUppercase\bibname}}%
      \list{\@biblabel{\@arabic\c@enumiv}}%
           {\settowidth\labelwidth{\@biblabel{#1}}%
            \leftmargin\labelwidth
            \advance\leftmargin\labelsep
            \@openbib@code
            \usecounter{enumiv}%
            \let\p@enumiv\@empty
            \renewcommand\theenumiv{\@arabic\c@enumiv}}%
      \sloppy
      \clubpenalty4000
      \@clubpenalty \clubpenalty
      \widowpenalty4000%
      \sfcode`\.\@m}
     {\def\@noitemerr
       {\@latex@warning{Empty `thebibliography' environment}}%
      \endlist}
\newcount\hour \newcount\minute
\hour=\time  \divide \hour by 60
\minute=\time
\loop \ifnum \minute > 59 \advance \minute by -60 \repeat
\def\thesistime{\ifnum \hour<13 \number\hour:%
                      \ifnum \minute<10 0\fi
                      \number\minute
                      \ifnum \hour<12 \ am\else \ pm\fi
         \else \advance \hour by -12 \number\hour:%
                      \ifnum \minute<10 0\fi
                      \number\minute \ pm\fi}
\def\timestamp{\thesistime, \today}
\newcommand{\@uppercase}[1]{{\expandafter\uppercase\expandafter{#1}}}
\newcommand\@separator{\vrule height.4pt depth0pt width3.5in}
\newcommand\@numberword[1]{\ifcase#1 zero\or one\or two\or three\or four\or five\or six\or seven\or eight\else nine\fi\relax}
\endinput
%%
%% End of file `cssethesis.cls'.
