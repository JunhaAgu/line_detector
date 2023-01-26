#include <opencv2/opencv.hpp>
#include <time.h>

using namespace cv;
using namespace std;

int main(int, char**)
{
	clock_t start, end;

	double minVal, maxVal;
	Point minLoc, maxLoc;
	Point matchLoc;

	Mat src = imread("/home/junhakim/original.png");
	if (src.empty())
	{
		cout << "no src Image!"<< endl;
		return -1;
	}

	Mat templ = imread("/home/junhakim/template.png");
	if (src.empty())
	{
		cout << "no templ Image!" << endl;
		return -1;
	}

	Mat result;

	for (int i = 0; i < 6; i++)
	{
		Mat img_out;
		src.copyTo(img_out);

		int Matching_method = i;
		/*
		0: TM_SQDIFF
		1: TM_SQDIFF NORMED
		2: TM CCORR
		3: TM CCORR NORMED
		4: TM COEFF
		5: TM COEFF NORMED";
		*/

		start = clock();


		matchTemplate(src, templ, result, i);
		normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
		if (Matching_method == 0 || Matching_method == 1)
		{
			matchLoc = minLoc;
		}
		else
			matchLoc = maxLoc;


		end = clock();

		cout << "Searching time : " << difftime(end, start) / CLOCKS_PER_SEC << endl;

		cout << "Min value : " << minVal << endl;

		cout << "Max value : " << maxVal << endl;

		rectangle(img_out, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar(0, 0, 255), 1);

		cvtColor(result, result, COLOR_GRAY2BGR);
		circle(result, matchLoc, 3, Scalar(0, 0, 255), 1);

		imshow("src", img_out);
		imshow("templ", templ);
		imshow("result", result);

		waitKey(0);

	}
	return 0;
}