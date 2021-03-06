% Package for a Diploma Thesis at CDS-ACIN
% ========================================
%
% DO NOT EDIT THIS FILE!
%
% -----------------------------------------
%
% v1.0: 11.03.2009, Thomas K�nig
% 		26.05.2009, Thomas K�nig:
% 			- Kopfzeile wird jetzt bei der Berechnung des Satzspiegels als zum Text
%				geh�rig betrachtet. Dadurch wird der obere Rand gr��er.
%			- Bindekorrektur von 10mm hinzugef�gt
%       25.06.2009, Thomas K�nig
%			- In Vorwort etc umgestellt von \chapter* auf \addchap*.
%				Dieser Befehl l�scht die Kolumnentitel, verhindert also, dass evtl. falsche
%				Angaben in der Kopfzeile stehen. Muss evtl. noch angepasst werden.
%				(\addchap (ohne Stern) kann z.Bsp. f�r eine nicht nummerierte Einleitung
%				verwendet werden.)
% v1.0.1: 5.01.2011, Thomas K�nig
%			- Ein zus�tzlicher 'pagestyle' (ChapterOnly) wurde definiert. Damit kann der
% 			  Stil der Kopfzeile ge�ndert werden, so dass nur der Name des Kapitels
% 			  angezeigt wird. Dies wird ben�tigt, wenn der Anfang eines Kapitels (der
% 			  Teil, bevor die erste section beginnt) sich �ber mehr als 2 Seiten
% 			  erstreckt. Dann wird mit dem normalen Seitenstil der Name des chapters
% 			  doppelt angezeigt. Um dies zu umgehen muss nun nach dem chapter-Befehl
% 			  der Befehl \pagestyle{ChapterOnly} und nach der ersten section der Befehl
% 			  \pagestyle{scrheadings} aufgerufen werden. (Prinzipiell k�nnte man wohl
% 			  einfach die Befehle chapter und section umdefinieren...)
% v1.1, Thomas K�nig
%			- Erkl�rung (Code of Conduct) hinzugef�gt
%			- siunitx als Standard-package verwenden (siunitx=false nur noch als option,
% 			  dann wird kein package f�r si-units eingebunden)
% 2013/03/07 added english title page
% pdfLaTeX Version, Georg Stadler, 13.11.2013
%           - Umstellung auf pdfLaTeX


% --- Characteristics ---
%=========================
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{ACIN_diploma_thesis}[2013/13/11 style for ACIN diploma thesis]


% --- Initialising ---
%======================

% --- declare options ---
%==========================


%================ automatic selection of language-dependent sections
\AtEndOfClass{
	\newboolean{LANGenglish}%
	\newboolean{LANGaustrian}
	\setboolean{LANGenglish}{true}%
	\setboolean{LANGaustrian}{false}
}
\DeclareOption{naustrian}{
	\PassOptionsToPackage{USenglish, ngerman, naustrian}{babel}
	\PassOptionsToClass{naustrian}{scrreprt}
	\AtEndOfClass{
		\setboolean{LANGenglish}{false}
		\setboolean{LANGaustrian}{true}
		\selectlanguage{naustrian}
	}
	\newcommand{\GerEng}[2]{#1}
}
\DeclareOption{ngerman}{
	\PassOptionsToPackage{USenglish, ngerman}{babel}
	\PassOptionsToClass{ngerman}{scrreprt}
	\AtEndOfClass{
	 	\setboolean{LANGenglish}{false}
	 	\selectlanguage{ngerman}
	}
	\newcommand{\GerEng}[2]{#1}
}
\DeclareOption{USenglish}{
	\PassOptionsToPackage{ngerman, USenglish}{babel}
	\PassOptionsToClass{USenglish}{scrreprt}
	\AtEndOfClass{
		\setboolean{LANGenglish}{true}
		\selectlanguage{USenglish}
	}
	\newcommand{\GerEng}[2]{#2}
}


\DeclareOption{Entwurf}{
	% 	\PassOptionsToPackage{timestamp, none, bottom, dark, draft}{draftcopy}
	% 	\AtEndOfClass{
	% 		\RequirePackage[dvips]{draftcopy}
	% 		\draftcopySetGrey{0.6}
	% 		\draftcopyName{Entwurf: \today}{50}
	% 	}

	% The above version doesn't work with pdflatex. Use the following instead:

	\AtEndOfClass{
		\usepackage{graphicx,type1cm,eso-pic,color} %LOOKUP

		\makeatletter
		\AddToShipoutPicture{%
		\setlength{\@tempdimb}{.5\paperwidth}%
		\setlength{\@tempdimc}{15mm}%
		\setlength{\unitlength}{1pt}%
		\put(\strip@pt\@tempdimb,\strip@pt\@tempdimc){%
		\makebox(0,0){\rotatebox{0}{\textcolor[gray]{0.6}%
		{\fontsize{1.7cm}{1.7cm}\selectfont{Entwurf: \today}}}}%
		}%
		}
		\makeatother
	}
}

\DeclareOption{draft}{
	\PassOptionsToClass{draft}{scrreprt}
}

\DeclareOption{PrintVersion}{
	\PassOptionsToPackage{pdfborder={0 0 0}}{hyperref}
}

\DeclareOption{NoTableOfContents}{\providecommand{\NoTableOfContents}{Dummy}}
\DeclareOption{NoListOfFigures}{\providecommand{\NoListOfFigures}{Dummy}}
\DeclareOption{NoListOfTables}{\providecommand{\NoListOfTables}{Dummy}}

\DeclareOption{siunitx=false}{
	\AtEndOfClass{
		\setboolean{siunitx}{false}
	}
}


% --- process options ---
%=========================

\ProcessOptions
\relax

% --- load class ---
%=========================

\PassOptionsToClass{
	fontsize=12pt,
	a4paper,
	twoside=false,
	DIV=10,		% changed to have more space above the headline
	BCOR=10mm,
	numbers=noenddot % keine Punkte nach letzter Nummer (z.B. 1.2.3 statt 1.2.3.), Problem tritt nur auf, sofern Anhaenge eingefuegt werden
}{scrreprt}

\LoadClass{scrreprt}

% --- load packages ---
%========================

%\PassOptionsToPackage{headinclude}{typearea} % ignored ?!?

\PassOptionsToPackage{
%pdf%	ps2pdf,							%% similar to dvips but redefines some macros for use with ps2pdf
	final,
	bookmarks=true,				%% if true, generate PDF bookmarks (requires two passes of pdflatex)
	bookmarksopen=false,			%% if true, show all PDF bookmarks expanded
	bookmarksnumbered=true,		%% if true, add the section numbers to the bookmarks
		linkbordercolor=acin_red,
		urlbordercolor=acin_red,
		citebordercolor=acin_red,
		filebordercolor=acin_red,
	pdfpagemode=UseOutlines, %% None, UseOutlines (=show bookmarks), UseThumbs (show thumbnails), FullScreen
	breaklinks,
	plainpages=false,
	hyperindex=true
}{hyperref}

\AtBeginDocument{
\hypersetup{pdfauthor = {\ACIN@PDFauthor},%
	pdftitle = {\ACIN@PDFtitle},%
	pdfsubject = {\ACIN@PDFsubject}}
}

\RequirePackage{ifthen}
\newboolean{siunitx}
\setboolean{siunitx}{true}


% packages needed for positioning of the boxes on the title page:
\RequirePackage[absolute, overlay]{textpos} %, showboxes
\RequirePackage{calc}


% packages for fonts:
%pdf%\RequirePackage{ae}
%pdf%\RequirePackage{aecompl}

% Now, it would be better to load {ucs} here and then us [utf8x] for input
% encoding. At the moment input files must be in CP-1252 (ansi) encoding.
\RequirePackage[ansinew]{inputenc}
\RequirePackage[T1]{fontenc}
\RequirePackage{lmodern}
\RequirePackage{textcomp}

\RequirePackage{babel}
\RequirePackage{csquotes}	%to avoid biblatex warning


%% packages for graphics
%\RequirePackage{float}
\RequirePackage{graphicx}
\RequirePackage[permil]{overpic}

\RequirePackage{placeins} % places floating objects before new section
% starts, provides \FloatBarrier which will place all remaining floating
% objects.
\RequirePackage[hyperref]{xcolor}

% packages for math stuff
\RequirePackage{amssymb}
\RequirePackage{amsmath} %[tbtags]
\RequirePackage{amsfonts}
\RequirePackage{bbm}
\RequirePackage{mathtools} % z.B. Matrixeintraege schoener setzen
\RequirePackage{icomma}

\AtEndOfClass{
	\ifthenelse{\boolean{siunitx}}{
		\RequirePackage{siunitx}
		\sisetup{
			per-mode=symbol,
			exponent-product = \cdot
		}
	}{}
}

\RequirePackage{xspace}


\RequirePackage{prettyref}

%\RequirePackage[a4paper,includehead,top=2cm]{geometry}

% packages for layout
\RequirePackage[automark]{scrpage2}		% Kopf-/Fusszeilen f�r KOMA-Klassen
\RequirePackage{microtype} % optischer Randausgleich (nur pdflatex)


\AtEndOfClass{
	% erweiterte Optionen f�r den Umgang mit dem Literaturverzeichnis
	%\RequirePackage[square,sort,compress,numbers]{natbib}
	\RequirePackage[backend=bibtex,
			%refsection=chapter,
			%citereset=chapter,
			style=ieee,
			sortcites,
			url=true,
			doi=false,
			defernumbers
			]{biblatex}[2012/08/17]
	\addbibresource{bibliography.bib}
	
	\RequirePackage{hyperref}
	\RequirePackage[all]{hypcap} % Damit werden die Links auf ein figure nicht mehr auf die Bildunterschrift sondern auf den Begin der Umgebung gesetzt. (must be loaded after hyperref)
}





% --- main part ---
%===================

%pdf%\DeclareGraphicsExtensions{.eps}

% general macros
%==================

\input{ACIN_diploma_GeneralMacros}
\input{ACIN_diploma_macros}

\input{ACIN_diploma_colors}


\setcounter{secnumdepth}{2}
\setcounter{tocdepth}{3}


% ++++++++++++++++++++++++++++++
% +														 +
% + prettyref-Definitionen 		 +
% + 													 +
% ++++++++++++++++++++++++++++++

\GerEng{
	\newrefformat{fig}{Abbildung~\ref{#1}}
	\newrefformat{eq}{(\ref{#1})}
	\newrefformat{apx}{Anhang~\ref{#1}}
	\newrefformat{cha}{Kapitel~\ref{#1}}
	\newrefformat{sec}{Abschnitt~\ref{#1}}
	\newrefformat{sub}{Abschnitt~\ref{#1}}
	\newrefformat{tab}{Tabelle~\ref{#1}}
	\newrefformat{def}{Definition~\ref{#1}}
}{
	\newrefformat{fig}{Figure~\ref{#1}}
	\newrefformat{eq}{(\ref{#1})}
	\newrefformat{apx}{Appendix~\ref{#1}}
	\newrefformat{cha}{Chapter~\ref{#1}}
	\newrefformat{sec}{Section~\ref{#1}}
	\newrefformat{sub}{Section~\ref{#1}}
	\newrefformat{tab}{Table~\ref{#1}}
	\newrefformat{def}{Definition~\ref{#1}}
	\newrefformat{lst}{Listing~\ref{#1}}
}


% +++++++++++++++++++++++++++
% + 												+
% + Kopf- und Fu�zeilen 		+
% + 												+
% +++++++++++++++++++++++++++


%\pagestyle{scrheadings}
% umdefinieren der 'normalen' Kopf-/Fusszeilen:
% ihead: innere Kopfzeile, ohead: �u�ere Kopfzeile, cfoot: mittlere Fusszeile
% [Definition f�r scrplain]{Definition f�r scrheadings}
% scrplain wird bei Seiten mit neuem Kapitel verwendet, scrheadings sonst
\setheadsepline{.4pt}		% Linie zwischen Kopfzeile und Text
%\automark[section]{chapter}
%leftmark wird durch subsection, rightmark durch section aktualisiert
\clearscrheadfoot 		% l�sche bisherige Definition vollst�ndig
\ihead[]{\leftmark} 	%evtl. \thesection \thechapter etc. verwenden
\ohead[]{\normalfont\rightmark\hspace*{0.5cm}\textbf\thepage}
\cfoot[\rmfamily\thepage]{}

\automark[section]{chapter}

\newpagestyle{Preface} % neuer Seitenstil
  %Seitenkopf:
	{
		{\headmark\hfill\textbf\thepage} % Gerade Seite
		{\textbf\thepage\hfill\headmark} % Ungerade Seite
		{\headmark\hfill\textbf\thepage} % Einseitiges Layout
	}
	%Seitenfuss:
	{
		{\hfill}
		{\hfill}
		{\hfill}
	}
	
\newpagestyle{ChapterOnly} % neuer Seitenstil
  %Seitenkopf:
	{
		{\headmark\hfill\textbf\thepage} % Gerade Seite
		{\textbf\thepage\hfill\headmark} % Ungerade Seite
		{\headmark\hfill\textbf\thepage} % Einseitiges Layout
	}
	%Seitenfuss:
	{
		{\hfill\rmfamily\thepage\hfill}
		{\hfill\rmfamily\thepage\hfill}
		{\hfill\rmfamily\thepage\hfill}
	}
	
%
% %----- modify pagesetup and headings
% % \textheight 22cm
% % \textwidth 16cm
% % \oddsidemargin 8mm
% % \topmargin 0pt
% % \headheight 17pt


% ======================
% ||   hyphen rules   ||
% ======================

	\IfFileExists{hyphen_german.tex}{
		\begin{otherlanguage*}{ngerman}
			{\input{hyphen_german.tex}}
		\end{otherlanguage*}
		\begin{otherlanguage*}{naustrian}
			{\input{hyphen_german.tex}}
		\end{otherlanguage*}
	}{
		\ClassWarningNoLine{ACIN_diploma_thesis}{File hyphen_german.tex does not exist!}
	}
	\IfFileExists{hyphen_english.tex}{
		\begin{otherlanguage*}{USenglish}{
			{\input{hyphen_english.tex}}
		}\end{otherlanguage*}
	}{
		\ClassWarningNoLine{ACIN_diploma_thesis}{File hyphen_english.tex does not exist!}
	}


\input{ACIN_diploma_Title}

