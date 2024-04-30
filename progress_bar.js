Qualtrics.SurveyEngine.addOnload(function()
{
	/*Place your JavaScript here to run when the page loads*/
	let progress = Qualtrics.SurveyEngine.getJSEmbeddedData("Progress"); // Retrieve existing progress
    
    const progressBarValue = (progress / 16) * 100; // Calculate progress dynamically
    
    const progressBarFill = document.querySelector("#progress-bar-fill > div");
    progressBarFill.style.width = progressBarValue + "%"; // Set width of the progress bar's inner div
    
    const progressBarPercent = document.querySelector("#progress-bar-percent");
    progressBarPercent.textContent = Math.round(progressBarValue) + "%"; // Update content of the progress bar percent span
    
	progress++;
	Qualtrics.SurveyEngine.setJSEmbeddedData("Progress", progress); // Update progress data
});

Qualtrics.SurveyEngine.addOnReady(function()
{
	/*Place your JavaScript here to run when the page is fully displayed*/

});

Qualtrics.SurveyEngine.addOnUnload(function()
{
	/*Place your JavaScript here to run when the page is unloaded*/
	
});