#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<unistd.h>

int main(void)
{

char keywords[15][15] = {"garbage","trash", "plastic","trash","can"};

		printf("Capturing and Analyzing image. This will take some time \n");
		sleep(3);
		system("fswebcam -o pic.jpg");
		sleep(2);
		printf("webcam step over \n");
		system("python /home/pi/Testing/imageClassifier/classify_image.py --image=/home/pi/Testing/imageClassifier/trash.jpg --model_dir=/home/pi/Testing/imageClassifier/imagenet/ >classified.txt"); 
		sleep(2);
		
		// see if result contains keyword		
		// send coordinates if found
		char msg[20] = {0};
		char *buffer = 0;
		long length;
		FILE *f = fopen ("garb.txt", "rb");
		
		char found = 0;
		
		if(f)
		{
		  fseek(f, 0, SEEK_END);
		  length = ftell(f);
		  fseek(f, 0, SEEK_SET);
		  buffer = malloc(length);
		  if(buffer)
		  {
			fread(buffer, 1, length, f);
		  }
		  fclose(f);
		}

		if(buffer)
		{
			for(int k = 0; k < 15; k++)
			{
				if(keywords[k][0] != 0)
				{
					printf("%s \n", keywords[k]);

					if(strstr(buffer, keywords[k]) != NULL)
					{
						strncpy(msg, "Found Garbage", sizeof(msg));
						printf("%s \n", msg);
					}
				}
			}
		}
}
